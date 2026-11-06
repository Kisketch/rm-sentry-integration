#include "autoaim/pose.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>

namespace autoaim {

namespace {
// camera optical (X right, Y down, Z forward) -> world (x forward, y left, z up)
const cv::Matx33d kWorldFromOptical(0, 0, 1, -1, 0, 0, 0, -1, 0);

cv::Matx33d rotZ(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {c, -s, 0, s, c, 0, 0, 0, 1};
}
cv::Matx33d rotY(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {c, 0, s, 0, 1, 0, -s, 0, c};
}
cv::Matx33d rotX(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {1, 0, 0, 0, c, -s, 0, s, c};
}
}  // namespace

CameraModel::CameraModel(const CameraConfig& cfg) {
    K_ = cfg.K();
    dist_ = cv::Mat(cfg.dist, true).reshape(1, 1);
    // Default extrinsic: the old video convention, with `pitch_deg` as the
    // camera tilt.  A measured robot supplies R_cam2gimbal explicitly.
    const cv::Matx33d from_pitch = rotY(cfg.pitch_deg * kPi / 180.0) * kWorldFromOptical;
    if (cfg.hasExtrinsicRotation()) {
        R_cg_ = cv::Matx33d(cfg.R_cam2gimbal.data());
    } else {
        R_cg_ = from_pitch;
    }
    if (cfg.t_cam2gimbal.size() == 3)
        t_cg_ = {cfg.t_cam2gimbal[0], cfg.t_cam2gimbal[1], cfg.t_cam2gimbal[2]};
}

void CameraModel::setGimbalAttitude(double yaw, double pitch, double roll) {
    // Old sentry convention: R_gimbal2world = Rz(yaw) * Ry(-pitch) * Rx(roll),
    // with positive pitch looking down.
    R_wg_ = rotZ(yaw) * rotY(-pitch) * rotX(roll);
}

cv::Vec3d CameraModel::cameraToWorld(const cv::Vec3d& p_cam) const {
    return R_wg_ * (R_cg_ * p_cam + t_cg_);
}

cv::Vec3d CameraModel::worldToCamera(const cv::Vec3d& p_world) const {
    return R_cg_.t() * (R_wg_.t() * p_world - t_cg_);
}

bool CameraModel::inFront(const cv::Vec3d& world) const {
    return worldToCamera(world)[2] > 0.05;
}

cv::Point2d CameraModel::project(const cv::Vec3d& world) const {
    const cv::Vec3d c = worldToCamera(world);
    std::vector<cv::Point3d> obj{{c[0], c[1], c[2]}};
    std::vector<cv::Point2d> img;
    cv::projectPoints(obj, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K_, dist_, img);
    return img[0];
}

cv::Matx33d armorRotation(double yaw, double pitch) { return rotZ(yaw) * rotY(pitch); }

std::array<cv::Vec3d, 4> armorCorners(const cv::Vec3d& center, double yaw, double pitch,
                                      ArmorType type) {
    const ArmorGeometry& g = geometryOf(type);
    const double hw = g.light_width / 2, hh = g.light_height / 2;
    // armor frame: x into the robot, y to the left (seen from the camera), z up
    const cv::Vec3d local[4] = {{0, hw, -hh}, {0, hw, hh}, {0, -hw, hh}, {0, -hw, -hh}};
    const cv::Matx33d R = armorRotation(yaw, pitch);
    std::array<cv::Vec3d, 4> out;
    for (int i = 0; i < 4; ++i) out[i] = center + R * local[i];
    return out;
}

PoseSolver::PoseSolver(const CameraModel& camera, const PoseConfig& cfg)
    : camera_(camera), cfg_(cfg) {}

double PoseSolver::reprojectionError(const ArmorDetection& det, const cv::Vec3d& center, double yaw,
                                     double pitch) const {
    const auto corners = armorCorners(center, yaw, pitch, det.type);
    double err = 0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point2d p = camera_.project(corners[i]);
        const cv::Point2d d = p - cv::Point2d(det.corners[i]);
        err += std::sqrt(d.dot(d));
    }
    return err / 4.0;
}

bool PoseSolver::solve(const ArmorDetection& det, ArmorObservation& out) const {
    // The classifier runs before the pose, so the outpost can be forced onto the
    // large-armor geometry even if its aspect ratio was ambiguous.
    ArmorDetection d = det;
    if (d.number == "outpost" && cfg_.outpost_as_large) d.type = ArmorType::OUTPOST;
    const ArmorGeometry& g = geometryOf(d.type);
    const double hw = g.light_width / 2, hh = g.light_height / 2;
    const std::vector<cv::Point3d> obj{{0, hw, -hh}, {0, hw, hh}, {0, -hw, hh}, {0, -hw, -hh}};
    const std::vector<cv::Point2d> img{cv::Point2d(d.corners[0]), cv::Point2d(d.corners[1]),
                                       cv::Point2d(d.corners[2]), cv::Point2d(d.corners[3])};
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errs;
    cv::Mat rv, tv;
    int n = 0;
    try {
        n = cv::solvePnPGeneric(obj, img, camera_.K(), camera_.dist(), rvecs, tvecs, false,
                                cv::SOLVEPNP_IPPE, rv, tv, errs);
    } catch (const cv::Exception&) {
        return false;
    }
    if (n <= 0) return false;

    const double expected_pitch =
        (d.type == ArmorType::OUTPOST ? cfg_.outpost_armor_pitch_deg : cfg_.armor_pitch_deg) *
        kPi / 180.0;
    int best = 0;
    double best_cost = 1e18;
    double best_yaw = 0;
    cv::Vec3d best_t;
    for (int i = 0; i < n; ++i) {
        cv::Matx33d R_ca;
        cv::Rodrigues(rvecs[i], R_ca);
        const cv::Matx33d R_wa = camera_.R_world_cam() * R_ca;
        const double yaw = std::atan2(R_wa(1, 0), R_wa(0, 0));
        const double pitch = std::asin(std::clamp(-R_wa(2, 0), -1.0, 1.0));
        const cv::Vec3d t_c(tvecs[i].at<double>(0), tvecs[i].at<double>(1), tvecs[i].at<double>(2));
        const double err = errs.size() > static_cast<std::size_t>(i) ? errs[i] : 0.0;
        // reprojection error dominates; pitch prior breaks the IPPE near-tie
        const double cost = err + 2.0 * std::abs(pitch - expected_pitch);
        if (cost < best_cost) {
            best_cost = cost, best = i, best_yaw = yaw;
            best_t = camera_.cameraToWorld(t_c);
        }
    }
    (void)best;

    out.det = d;
    out.position = best_t;
    out.distance = cv::norm(best_t);
    out.yaw = best_yaw;

    if (cfg_.optimize_yaw) {
        // Fixed pitch, search yaw around the line of sight.
        const double los = std::atan2(best_t[1], best_t[0]);
        const double range = cfg_.yaw_search_range_deg * kPi / 180.0;
        double y_best = best_yaw, e_best = 1e18;
        for (double dy = -range; dy <= range + 1e-9; dy += kPi / 180.0) {
            const double e = reprojectionError(d, best_t, los + dy, expected_pitch);
            if (e < e_best) e_best = e, y_best = los + dy;
        }
        // golden-section refinement within +-1 deg
        double a = y_best - kPi / 180.0, b = y_best + kPi / 180.0;
        const double gr = 0.6180339887498949;
        double c = b - gr * (b - a), dd = a + gr * (b - a);
        double fc = reprojectionError(d, best_t, c, expected_pitch);
        double fd = reprojectionError(d, best_t, dd, expected_pitch);
        for (int it = 0; it < 20; ++it) {
            if (fc < fd) {
                b = dd, dd = c, fd = fc;
                c = b - gr * (b - a);
                fc = reprojectionError(d, best_t, c, expected_pitch);
            } else {
                a = c, c = dd, fc = fd;
                dd = a + gr * (b - a);
                fd = reprojectionError(d, best_t, dd, expected_pitch);
            }
        }
        out.yaw = wrapAngle(0.5 * (a + b));
        out.reproj_error = reprojectionError(d, best_t, out.yaw, expected_pitch);
    } else {
        out.reproj_error = reprojectionError(d, best_t, out.yaw, expected_pitch);
    }

    if (cfg_.yaw_gain != 1.0) {
        const double los = std::atan2(best_t[1], best_t[0]);
        out.yaw = wrapAngle(los + cfg_.yaw_gain * wrapAngle(out.yaw - los));
    }

    const cv::Point2d c(camera_.K()(0, 2), camera_.K()(1, 2));
    out.image_center_dist = cv::norm(cv::Point2d(det.center) - c);
    return std::isfinite(out.position[0]) && out.distance > 0.2 && out.distance < 30.0;
}

std::vector<ArmorObservation> PoseSolver::solveAll(const std::vector<ArmorDetection>& dets) const {
    std::vector<ArmorObservation> out;
    for (const auto& d : dets) {
        ArmorObservation o;
        if (solve(d, o)) out.push_back(o);
    }
    return out;
}

}  // namespace autoaim
