#include "autoaim/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoaim {

const char* toString(TrackState s) {
    switch (s) {
        case TrackState::LOST: return "LOST";
        case TrackState::DETECTING: return "DETECTING";
        case TrackState::TRACKING: return "TRACKING";
        case TrackState::TEMP_LOST: return "TEMP_LOST";
    }
    return "?";
}

int chooseInitialTarget(const std::vector<ArmorObservation>& armors) {
    int best = -1;
    double best_d = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(armors.size()); ++i)
        if (armors[i].image_center_dist < best_d) best_d = armors[i].image_center_dist, best = i;
    return best;
}

namespace {

// Anisotropic position covariance: tight across the viewing ray, loose along it.
cv::Matx33d positionNoise(const ArmorObservation& obs, double fx, double px) {
    const double d = std::max(obs.distance, 0.3);
    const cv::Vec3d u = obs.position / d;
    const double bar_px = std::max(4.0, 0.5 * (obs.det.left.length + obs.det.right.length));
    const double s_lat = d * px / fx;
    const double s_dep = d * px / bar_px;
    cv::Matx33d R = cv::Matx33d::eye() * (s_lat * s_lat);
    const double extra = s_dep * s_dep - s_lat * s_lat;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) += extra * u[i] * u[j];
    return R;
}

template <int M>
void addWhiteNoiseAccel(cv::Matx<double, M, M>& Q, int i, double q, double dt) {
    Q(i, i) += q * dt * dt * dt / 3.0;
    Q(i, i + 1) += q * dt * dt / 2.0;
    Q(i + 1, i) += q * dt * dt / 2.0;
    Q(i + 1, i + 1) += q * dt;
}

}  // namespace

// ============================================================ RobotTracker

RobotTracker::RobotTracker(const TrackerConfig& cfg, const CameraModel& camera)
    : cfg_(cfg), camera_(camera) {}

double RobotTracker::armorRadius(const Vec& x, int k) const {
    if (cfg_.armors_num == 4 && (k % 2) != 0) return x[R2];
    return x[R1];
}

double RobotTracker::armorYaw(const Vec& x, int k) const {
    return x[YAW] + 2.0 * kPi * k / cfg_.armors_num;
}

cv::Vec3d RobotTracker::armorPosition(const Vec& x, int k) const {
    const double phi = armorYaw(x, k), r = armorRadius(x, k);
    const double dz = (cfg_.armors_num == 4 && (k % 2) != 0) ? x[DZ] : 0.0;
    return {x[XC] - r * std::cos(phi), x[YC] - r * std::sin(phi), x[ZC] + dz};
}

RobotTracker::Vec RobotTracker::predict(double t) const {
    Vec x = x_;
    const double dt = t - stamp_;
    if (cfg_.singer_tau > 0) {
        const double a = std::exp(-dt / cfg_.singer_tau);
        const double b = cfg_.singer_tau * (1.0 - a);
        x[XC] += x[VXC] * b;
        x[YC] += x[VYC] * b;
        x[ZC] += x[VZC] * b;
        // The velocity decays too (same F as transition()).  Without this the
        // extrapolated state carried the undamped velocity, so a TargetState
        // taken at t > stamp_ and extrapolated again overshot by v*(1-a)*b.
        x[VXC] *= a;
        x[VYC] *= a;
        x[VZC] *= a;
    } else {
        x[XC] += x[VXC] * dt;
        x[YC] += x[VYC] * dt;
        x[ZC] += x[VZC] * dt;
    }
    if (cfg_.ca_model > 0) {
        x[XC] += 0.5 * x[AXC] * dt * dt;
        x[YC] += 0.5 * x[AY] * dt * dt;
        x[VXC] += x[AXC] * dt;
        x[VYC] += x[AY] * dt;
    }
    x[YAW] += x[W] * dt;
    return x;
}

void RobotTracker::init(const ArmorObservation& obs, double stamp) {
    const double r = cfg_.init_radius;
    x_ = Vec::all(0);
    x_[XC] = obs.position[0] + r * std::cos(obs.yaw);
    x_[YC] = obs.position[1] + r * std::sin(obs.yaw);
    x_[ZC] = obs.position[2];
    x_[YAW] = obs.yaw;
    x_[R1] = r;
    x_[R2] = r;
    x_[DZ] = 0;
    const double d = obs.distance;
    const double p0[N] = {0.05 * d, 1.0, 0.05 * d, 1.0, 0.05, 0.5, 0.3, 3.0, 0.08, 0.08, 0.05, 3.0, 3.0};
    P_ = Mat::zeros();
    for (int i = 0; i < N; ++i) P_(i, i) = p0[i] * p0[i];
    stamp_ = stamp;
    last_match_stamp_ = stamp;
    id_ = deviceId(obs.det);
    label_ = obs.det.number;
    armor_type = obs.det.type;
    state_ = TrackState::DETECTING;
    detect_count_ = 0;
    unmatched_with_target_ = 0;
    nis_ema_ = 3.0;
    q_scale_ = 1.0;
}

void RobotTracker::transition(double dt, Mat& F, Mat& Q) const {
    F = Mat::eye();
    if (cfg_.singer_tau > 0) {
        // Singer: velocity decays with tau instead of holding constant.
        const double a = std::exp(-dt / cfg_.singer_tau);
        const double b = cfg_.singer_tau * (1.0 - a);
        F(XC, VXC) = F(YC, VYC) = F(ZC, VZC) = b;
        F(VXC, VXC) = F(VYC, VYC) = F(VZC, VZC) = a;
    } else {
        F(XC, VXC) = F(YC, VYC) = F(ZC, VZC) = dt;
    }
    F(YAW, W) = dt;
    if (cfg_.ca_model > 0) {
        F(XC, AXC) = 0.5 * dt * dt;
        F(VXC, AXC) = dt;
        F(YC, AY) = 0.5 * dt * dt;
        F(VYC, AY) = dt;
    }
    Q = Mat::zeros();
    const double qxyz = cfg_.q_xyz * q_scale_;
    addWhiteNoiseAccel(Q, XC, qxyz, dt);
    addWhiteNoiseAccel(Q, YC, qxyz, dt);
    addWhiteNoiseAccel(Q, ZC, qxyz * 0.25, dt);
    addWhiteNoiseAccel(Q, YAW, cfg_.q_yaw, dt);
    if (cfg_.ca_model > 0) {
        Q(AXC, AXC) += cfg_.q_acc * dt;
        Q(AY, AY) += cfg_.q_acc * dt;
    }
    Q(R1, R1) += cfg_.q_r * dt;
    Q(R2, R2) += cfg_.q_r * dt;
    Q(DZ, DZ) += cfg_.q_dz * dt;
}

void RobotTracker::predictTo(double t) {
    const double dt = t - stamp_;
    if (dt <= 0) return;
    Mat F, Q;
    transition(dt, F, Q);
    x_ = F * x_;
    P_ = F * P_ * F.t() + Q;
    stamp_ = t;
}

RobotTracker::Mat RobotTracker::predictCovariance(double t, bool with_process_noise) const {
    const double dt = t - stamp_;
    if (dt <= 0) return P_;
    Mat F, Q;
    transition(dt, F, Q);
    return with_process_noise ? Mat(F * P_ * F.t() + Q) : Mat(F * P_ * F.t());
}

cv::Matx<double, 3, RobotTracker::N> RobotTracker::armorJacobian(const Vec& x, int k) const {
    const double phi = armorYaw(x, k), r = armorRadius(x, k);
    const bool second = cfg_.armors_num == 4 && (k % 2) != 0;
    const int ri = second ? R2 : R1;
    cv::Matx<double, 3, N> H = cv::Matx<double, 3, N>::zeros();
    H(0, XC) = 1, H(0, YAW) = r * std::sin(phi), H(0, ri) = -std::cos(phi);
    H(1, YC) = 1, H(1, YAW) = -r * std::cos(phi), H(1, ri) = -std::sin(phi);
    H(2, ZC) = 1;
    if (second) H(2, DZ) = 1;
    return H;
}

cv::Matx<double, 4, 4> RobotTracker::measurementNoise(const ArmorObservation& obs) const {
    cv::Matx<double, 4, 4> R = cv::Matx<double, 4, 4>::zeros();
    const cv::Matx33d Rp = positionNoise(obs, camera_.fx(), cfg_.pixel_noise);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = Rp(i, j);
    const double los = std::atan2(obs.position[1], obs.position[0]);
    const double c = std::max(0.0, std::cos(wrapAngle(obs.yaw - los)));
    R(3, 3) = cfg_.r_yaw + cfg_.r_yaw_head_on * std::pow(c, 6);
    return R;
}

bool RobotTracker::updateWith(const ArmorObservation& obs, int k, double& nis_pos) {
    const double phi = armorYaw(x_, k), r = armorRadius(x_, k);
    const bool second_pair = cfg_.armors_num == 4 && (k % 2) != 0;
    const int ri = second_pair ? R2 : R1;

    cv::Matx<double, 4, N> H = cv::Matx<double, 4, N>::zeros();
    H(0, XC) = 1;
    H(0, YAW) = r * std::sin(phi);
    H(0, ri) = -std::cos(phi);
    H(1, YC) = 1;
    H(1, YAW) = -r * std::cos(phi);
    H(1, ri) = -std::sin(phi);
    H(2, ZC) = 1;
    if (second_pair) H(2, DZ) = 1;
    H(3, YAW) = 1;

    const cv::Vec3d pa = armorPosition(x_, k);
    cv::Vec4d y(obs.position[0] - pa[0], obs.position[1] - pa[1], obs.position[2] - pa[2],
                wrapAngle(obs.yaw - phi));
    const cv::Matx<double, 4, 4> R = measurementNoise(obs);
    const cv::Matx<double, 4, 4> S = H * P_ * H.t() + R;
    const cv::Matx<double, 4, 4> Si = S.inv(cv::DECOMP_CHOLESKY);
    last_nis = (y.t() * Si * y)(0);
    {
        cv::Matx33d S3;
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) S3(a, b) = S(a, b);
        const cv::Vec3d y3(y[0], y[1], y[2]);
        nis_pos = (y3.t() * S3.inv(cv::DECOMP_CHOLESKY) * y3)(0);
    }
    const cv::Matx<double, N, 4> K = P_ * H.t() * Si;
    x_ += K * y;
    const Mat IKH = Mat::eye() - K * H;
    P_ = IKH * P_ * IKH.t() + K * R * K.t();  // Joseph form
    clampState();
    return true;
}

void RobotTracker::clampState() {
    x_[R1] = std::clamp(x_[R1], cfg_.min_radius, cfg_.max_radius);
    x_[R2] = std::clamp(x_[R2], cfg_.min_radius, cfg_.max_radius);
    x_[DZ] = std::clamp(x_[DZ], -0.15, 0.15);
    x_[W] = std::clamp(x_[W], -20.0, 20.0);
    x_[AXC] = std::clamp(x_[AXC], -15.0, 15.0);
    x_[AY] = std::clamp(x_[AY], -15.0, 15.0);
    // keep yaw bounded; every comparison uses wrapped differences
    x_[YAW] = wrapAngle(x_[YAW]);
}

void RobotTracker::update(const FrameObservations& frame) {
    matched_count = 0;
    if (state_ == TrackState::LOST) {
        const int idx = chooseInitialTarget(frame.armors);
        if (idx >= 0) init(frame.armors[idx], frame.stamp);
        return;
    }

    predictTo(frame.stamp);

    // Associate every same-id observation with one of the N predicted armors.
    struct Pair {
        int obs, k;
        double d2;
    };
    std::vector<Pair> pairs;
    bool target_seen = false;
    for (int i = 0; i < static_cast<int>(frame.armors.size()); ++i) {
        const auto& o = frame.armors[i];
        if (deviceId(o.det) != id_) continue;
        target_seen = true;
        const cv::Matx<double, 4, 4> R = measurementNoise(o);
        for (int k = 0; k < cfg_.armors_num; ++k) {
            const cv::Vec3d pa = armorPosition(x_, k);
            const cv::Vec3d dp = o.position - pa;
            const double dyaw = std::abs(wrapAngle(o.yaw - armorYaw(x_, k)));
            if (cv::norm(dp) > cfg_.max_match_distance * std::max(1.0, o.distance / 3.0) ||
                dyaw > cfg_.max_match_yaw_diff)
                continue;
            // position-only Mahalanobis distance
            const double phi = armorYaw(x_, k), r = armorRadius(x_, k);
            const int ri = (cfg_.armors_num == 4 && k % 2) ? R2 : R1;
            cv::Matx<double, 3, N> H = cv::Matx<double, 3, N>::zeros();
            H(0, XC) = 1, H(0, YAW) = r * std::sin(phi), H(0, ri) = -std::cos(phi);
            H(1, YC) = 1, H(1, YAW) = -r * std::cos(phi), H(1, ri) = -std::sin(phi);
            H(2, ZC) = 1;
            if (cfg_.armors_num == 4 && k % 2) H(2, DZ) = 1;
            cv::Matx33d S = H * P_ * H.t();
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b) S(a, b) += R(a, b);
            const double d2 = (dp.t() * S.inv(cv::DECOMP_CHOLESKY) * dp)(0);
            pairs.push_back({i, k, d2 + 4.0 * dyaw * dyaw});
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d2 < b.d2; });
    std::vector<bool> used_obs(frame.armors.size(), false), used_k(cfg_.armors_num, false);
    double nis_sum = 0;
    for (const auto& p : pairs) {
        if (used_obs[p.obs] || used_k[p.k]) continue;
        if (p.d2 > 40.0) continue;  // gate
        used_obs[p.obs] = used_k[p.k] = true;
        double nis_pos = 0;
        updateWith(frame.armors[p.obs], p.k, nis_pos);
        nis_sum += nis_pos;
        ++matched_count;
    }
    const bool matched = matched_count > 0;
    if (matched) {
        last_match_stamp_ = frame.stamp;
        // adaptive process noise (manoeuvre detection on the position innovation)
        const double a = cfg_.nis_ema_alpha;
        nis_ema_ = (1 - a) * nis_ema_ + a * (nis_sum / matched_count);
        q_scale_ = std::clamp(nis_ema_ / 3.0, 1.0, std::max(1.0, cfg_.q_adapt_max));
    }

    // State machine (rm_auto_aim)
    switch (state_) {
        case TrackState::DETECTING:
            if (matched) {
                if (++detect_count_ >= cfg_.tracking_thres) state_ = TrackState::TRACKING;
            } else {
                state_ = TrackState::LOST;
            }
            break;
        case TrackState::TRACKING:
            if (!matched) state_ = TrackState::TEMP_LOST;
            break;
        case TrackState::TEMP_LOST:
            if (matched)
                state_ = TrackState::TRACKING;
            else if (frame.stamp - last_match_stamp_ > cfg_.lost_time_thres)
                state_ = TrackState::LOST;
            break;
        case TrackState::LOST: break;
    }

    // Divergence guard: the target is visible but nothing matches for a while.
    if (!matched && target_seen) {
        if (++unmatched_with_target_ > 3) state_ = TrackState::LOST;
    } else {
        unmatched_with_target_ = 0;
    }
    if (state_ == TrackState::LOST) {
        const int idx = chooseInitialTarget(frame.armors);
        if (idx >= 0) init(frame.armors[idx], frame.stamp);
    }
}

// ============================================================ TargetState

cv::Vec3d TargetState::centerAt(double t) const {
    const double dt = std::max(0.0, t - stamp);
    double b = dt;
    if (singer_tau > 0) b = singer_tau * (1.0 - std::exp(-dt / singer_tau));
    return center + velocity * b;
}

ArmorState TargetState::armorAt(int k, double t) const {
    if (k < 0 || k >= armors_num) return {};
    const double dt = std::max(0.0, t - stamp);
    const ArmorState& a = armors[k];
    const double phi = wrapAngle(a.yaw + omega * dt);
    ArmorState out = a;
    const cv::Vec3d c = centerAt(t);
    // Plate height follows the centre's vertical motion (it used to stay at the
    // snapshot height while centerAt() moved).
    out.position = {c[0] - a.radius * std::cos(phi), c[1] - a.radius * std::sin(phi),
                    a.position[2] + (c[2] - center[2])};
    out.yaw = phi;
    return out;
}

TargetState RobotTracker::targetState(double t) const {
    TargetState s;
    s.valid = state_ != TrackState::LOST;
    s.id = id_;
    s.label = label_;
    s.state = state_;
    s.armors_num = cfg_.armors_num;
    s.type = armor_type;
    s.stamp = t;
    s.singer_tau = cfg_.singer_tau;
    const Vec x = predict(t);
    s.center = {x[XC], x[YC], x[ZC]};
    s.velocity = {x[VXC], x[VYC], x[VZC]};
    s.yaw = x[YAW];
    s.omega = x[W];
    const Mat Pc = predictCovariance(t, false);
    const int n = std::min(4, cfg_.armors_num);
    for (int k = 0; k < n; ++k) {
        s.armors[k].position = armorPosition(x, k);
        s.armors[k].yaw = armorYaw(x, k);
        s.armors[k].radius = armorRadius(x, k);
        const auto J = armorJacobian(x, k);
        const cv::Matx33d S = J * Pc * J.t();
        const cv::Vec3d normal(-std::sin(s.armors[k].yaw), std::cos(s.armors[k].yaw), 0);
        s.armors[k].sigma_lat = std::sqrt(std::max(0.0, (normal.t() * S * normal)(0)));
    }
    return s;
}

// ============================================================ OutpostTracker

OutpostTracker::OutpostTracker(const TrackerConfig& cfg, const CameraModel& camera)
    : cfg_(cfg), camera_(camera) {}

cv::Vec3d OutpostTracker::armorPosition(const Vec& x, int k) const {
    const double phi = armorYaw(x, k), r = x[R];
    return {x[XC] - r * std::cos(phi), x[YC] - r * std::sin(phi), x[Z1 + std::clamp(k, 0, 2)]};
}

double OutpostTracker::armorYaw(const Vec& x, int k) const {
    return x[YAW] + 2.0 * kPi * k / 3.0;
}

double OutpostTracker::armorRadius(const Vec& x, int) const { return x[R]; }

OutpostTracker::Vec OutpostTracker::predict(double t) const {
    Vec x = x_;
    const double dt = t - stamp_;
    if (cfg_.singer_tau > 0) {
        const double a = std::exp(-dt / cfg_.singer_tau);
        const double b = cfg_.singer_tau * (1.0 - a);
        x[XC] += x[VXC] * b;
        x[YC] += x[VYC] * b;
        x[VXC] *= a;
        x[VYC] *= a;
    } else {
        x[XC] += x[VXC] * dt;
        x[YC] += x[VYC] * dt;
    }
    x[YAW] += x[W] * dt;
    return x;
}

void OutpostTracker::init(const ArmorObservation& obs, double stamp) {
    const double r = cfg_.outpost_init_radius;
    x_ = Vec::all(0);
    x_[XC] = obs.position[0] + r * std::cos(obs.yaw);
    x_[YC] = obs.position[1] + r * std::sin(obs.yaw);
    x_[Z1] = x_[Z2] = x_[Z3] = obs.position[2];
    x_[R] = r;
    x_[YAW] = obs.yaw;
    const double d = obs.distance;
    P_ = Mat::zeros();
    const double p0[N] = {0.05 * d, 1.0, 0.05 * d, 1.0, 0.06, 0.06, 0.06, 0.08, 0.4, 3.0};
    for (int i = 0; i < N; ++i) P_(i, i) = p0[i] * p0[i];
    stamp_ = last_match_stamp_ = stamp;
    id_ = obs.det.number.empty() ? "outpost" : obs.det.number;
    armor_type = ArmorType::OUTPOST;
    state_ = TrackState::DETECTING;
    detect_count_ = 0;
    unmatched_with_target_ = 0;
    nis_ema_ = 3.0;
    q_scale_ = 1.0;
}

void OutpostTracker::transition(double dt, Mat& F, Mat& Q) const {
    F = Mat::eye();
    if (cfg_.singer_tau > 0) {
        const double a = std::exp(-dt / cfg_.singer_tau);
        const double b = cfg_.singer_tau * (1.0 - a);
        F(XC, VXC) = F(YC, VYC) = b;
        F(VXC, VXC) = F(VYC, VYC) = a;
    } else {
        F(XC, VXC) = F(YC, VYC) = dt;
    }
    F(YAW, W) = dt;
    Q = Mat::zeros();
    const double qxyz = cfg_.q_xyz * q_scale_;
    addWhiteNoiseAccel(Q, XC, qxyz, dt);
    addWhiteNoiseAccel(Q, YC, qxyz, dt);
    addWhiteNoiseAccel(Q, YAW, cfg_.q_yaw, dt);
    for (int k = 0; k < 3; ++k) Q(Z1 + k, Z1 + k) += cfg_.q_outpost_z * dt;
    Q(R, R) += cfg_.q_outpost_r * dt;
}

void OutpostTracker::predictTo(double t) {
    const double dt = t - stamp_;
    if (dt <= 0) return;
    Mat F, Q;
    transition(dt, F, Q);
    x_ = F * x_;
    P_ = F * P_ * F.t() + Q;
    clampState();
    stamp_ = t;
}

OutpostTracker::Mat OutpostTracker::predictCovariance(double t, bool with_process_noise) const {
    const double dt = t - stamp_;
    if (dt <= 0) return P_;
    Mat F, Q;
    transition(dt, F, Q);
    return with_process_noise ? Mat(F * P_ * F.t() + Q) : Mat(F * P_ * F.t());
}

cv::Matx<double, 3, OutpostTracker::N> OutpostTracker::armorJacobian(const Vec& x, int k) const {
    const double phi = armorYaw(x, k), r = x[R];
    cv::Matx<double, 3, N> H = cv::Matx<double, 3, N>::zeros();
    H(0, XC) = 1, H(0, YAW) = r * std::sin(phi), H(0, R) = -std::cos(phi);
    H(1, YC) = 1, H(1, YAW) = -r * std::cos(phi), H(1, R) = -std::sin(phi);
    H(2, Z1 + std::clamp(k, 0, 2)) = 1;
    return H;
}

cv::Matx<double, 4, 4> OutpostTracker::measurementNoise(const ArmorObservation& obs) const {
    cv::Matx<double, 4, 4> R = cv::Matx<double, 4, 4>::zeros();
    const cv::Matx33d Rp = positionNoise(obs, camera_.fx(), cfg_.pixel_noise);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = Rp(i, j);
    const double los = std::atan2(obs.position[1], obs.position[0]);
    const double c = std::max(0.0, std::cos(wrapAngle(obs.yaw - los)));
    R(3, 3) = cfg_.r_yaw + cfg_.r_yaw_head_on * std::pow(c, 6);
    return R;
}

bool OutpostTracker::updateWith(const ArmorObservation& obs, int k, double& nis_pos) {
    const double phi = armorYaw(x_, k);
    const double r = x_[R];
    cv::Matx<double, 4, N> H = cv::Matx<double, 4, N>::zeros();
    H(0, XC) = 1, H(0, YAW) = r * std::sin(phi), H(0, R) = -std::cos(phi);
    H(1, YC) = 1, H(1, YAW) = -r * std::cos(phi), H(1, R) = -std::sin(phi);
    H(2, Z1 + std::clamp(k, 0, 2)) = 1;
    H(3, YAW) = 1;

    const cv::Vec3d pa = armorPosition(x_, k);
    cv::Vec4d y(obs.position[0] - pa[0], obs.position[1] - pa[1], obs.position[2] - pa[2],
                wrapAngle(obs.yaw - phi));
    const cv::Matx<double, 4, 4> R = measurementNoise(obs);
    const cv::Matx<double, 4, 4> S = H * P_ * H.t() + R;
    const cv::Matx<double, 4, 4> Si = S.inv(cv::DECOMP_CHOLESKY);
    last_nis = (y.t() * Si * y)(0);
    {
        cv::Matx33d S3;
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) S3(a, b) = S(a, b);
        const cv::Vec3d y3(y[0], y[1], y[2]);
        nis_pos = (y3.t() * S3.inv(cv::DECOMP_CHOLESKY) * y3)(0);
    }
    const cv::Matx<double, N, 4> K = P_ * H.t() * Si;
    x_ += K * y;
    const Mat IKH = Mat::eye() - K * H;
    P_ = IKH * P_ * IKH.t() + K * R * K.t();
    clampState();
    return true;
}

void OutpostTracker::clampState() {
    x_[R] = std::clamp(x_[R], cfg_.outpost_min_radius, cfg_.outpost_max_radius);
    x_[W] = std::clamp(x_[W], -20.0, 20.0);
    x_[YAW] = wrapAngle(x_[YAW]);
}

void OutpostTracker::update(const FrameObservations& frame) {
    matched_count = 0;
    if (state_ == TrackState::LOST) {
        for (const auto& o : frame.armors) {
            if (o.det.number == "outpost" || o.det.type == ArmorType::OUTPOST) {
                init(o, frame.stamp);
                break;
            }
        }
        return;
    }

    predictTo(frame.stamp);

    struct Pair {
        int obs, k;
        double d2;
    };
    std::vector<Pair> pairs;
    bool target_seen = false;
    for (int i = 0; i < static_cast<int>(frame.armors.size()); ++i) {
        const auto& o = frame.armors[i];
        if (o.det.number != id_ && o.det.type != ArmorType::OUTPOST) continue;
        target_seen = true;
        const cv::Matx<double, 4, 4> R = measurementNoise(o);
        for (int k = 0; k < 3; ++k) {
            const cv::Vec3d pa = armorPosition(x_, k);
            const cv::Vec3d dp = o.position - pa;
            const double dyaw = std::abs(wrapAngle(o.yaw - armorYaw(x_, k)));
            if (cv::norm(dp) > cfg_.max_match_distance * std::max(1.0, o.distance / 3.0) ||
                dyaw > cfg_.max_match_yaw_diff)
                continue;
            const auto H = armorJacobian(x_, k);
            cv::Matx33d S = H * P_ * H.t();
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b) S(a, b) += R(a, b);
            const double d2 = (dp.t() * S.inv(cv::DECOMP_CHOLESKY) * dp)(0);
            pairs.push_back({i, k, d2 + 4.0 * dyaw * dyaw});
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d2 < b.d2; });
    std::vector<bool> used_obs(frame.armors.size(), false), used_k(3, false);
    double nis_sum = 0;
    for (const auto& p : pairs) {
        if (used_obs[p.obs] || used_k[p.k] || p.d2 > 40.0) continue;
        used_obs[p.obs] = used_k[p.k] = true;
        double nis_pos = 0;
        updateWith(frame.armors[p.obs], p.k, nis_pos);
        nis_sum += nis_pos;
        ++matched_count;
    }
    const bool matched = matched_count > 0;
    if (matched) {
        last_match_stamp_ = frame.stamp;
        const double a = cfg_.nis_ema_alpha;
        nis_ema_ = (1 - a) * nis_ema_ + a * (nis_sum / matched_count);
        q_scale_ = std::clamp(nis_ema_ / 3.0, 1.0, std::max(1.0, cfg_.q_adapt_max));
    }
    switch (state_) {
        case TrackState::DETECTING:
            if (matched) {
                if (++detect_count_ >= cfg_.tracking_thres) state_ = TrackState::TRACKING;
            } else {
                state_ = TrackState::LOST;
            }
            break;
        case TrackState::TRACKING:
            if (!matched) state_ = TrackState::TEMP_LOST;
            break;
        case TrackState::TEMP_LOST:
            if (matched)
                state_ = TrackState::TRACKING;
            else if (frame.stamp - last_match_stamp_ > cfg_.lost_time_thres)
                state_ = TrackState::LOST;
            break;
        case TrackState::LOST: break;
    }
    if (!matched && target_seen) {
        if (++unmatched_with_target_ > 3) state_ = TrackState::LOST;
    } else {
        unmatched_with_target_ = 0;
    }
    if (state_ == TrackState::LOST) {
        for (const auto& o : frame.armors) {
            if (o.det.number == "outpost" || o.det.type == ArmorType::OUTPOST) {
                init(o, frame.stamp);
                break;
            }
        }
    }
}

TargetState OutpostTracker::targetState(double t) const {
    TargetState s;
    s.valid = state_ != TrackState::LOST;
    s.id = id_;
    s.label = "outpost";
    s.state = state_;
    s.armors_num = 3;
    s.type = ArmorType::OUTPOST;
    s.stamp = t;
    s.singer_tau = cfg_.singer_tau;
    const Vec x = predict(t);
    s.center = {x[XC], x[YC], x[Z1]};
    s.velocity = {x[VXC], x[VYC], 0};
    s.yaw = x[YAW];
    s.omega = x[W];
    const Mat Pc = predictCovariance(t, false);
    for (int k = 0; k < 3; ++k) {
        s.armors[k].position = armorPosition(x, k);
        s.armors[k].yaw = armorYaw(x, k);
        s.armors[k].radius = x[R];
        const auto J = armorJacobian(x, k);
        const cv::Matx33d S = J * Pc * J.t();
        const cv::Vec3d normal(-std::sin(s.armors[k].yaw), std::cos(s.armors[k].yaw), 0);
        s.armors[k].sigma_lat = std::sqrt(std::max(0.0, (normal.t() * S * normal)(0)));
    }
    return s;
}

TargetState snapshot(const RobotTracker& tracker, double t) { return tracker.targetState(t); }
TargetState snapshot(const OutpostTracker& tracker, double t) { return tracker.targetState(t); }

// ============================================================ PointTracker

PointTracker::PointTracker(const TrackerConfig& cfg, const CameraModel& camera)
    : cfg_(cfg), camera_(camera) {}

void PointTracker::init(const ArmorObservation& obs, double stamp) {
    x_ = Vec(obs.position[0], 0, obs.position[1], 0, obs.position[2], 0);
    P_ = Mat::zeros();
    const double d = obs.distance;
    P_(0, 0) = P_(2, 2) = 0.05 * d * 0.05 * d;
    P_(4, 4) = 0.05 * 0.05;
    P_(1, 1) = P_(3, 3) = P_(5, 5) = 1.0;
    stamp_ = last_match_ = stamp;
    id_ = deviceId(obs.det);
    yaw_ = obs.yaw;
    armor_type = obs.det.type;
    active_ = true;
}

PointTracker::Vec PointTracker::predict(double t) const {
    Vec x = x_;
    const double dt = t - stamp_;
    x[0] += x[1] * dt, x[2] += x[3] * dt, x[4] += x[5] * dt;
    return x;
}

void PointTracker::update(const FrameObservations& frame) {
    if (!active_) {
        const int idx = chooseInitialTarget(frame.armors);
        if (idx >= 0) init(frame.armors[idx], frame.stamp);
        return;
    }
    const double dt = frame.stamp - stamp_;
    Mat F = Mat::eye();
    F(0, 1) = F(2, 3) = F(4, 5) = dt;
    Mat Q = Mat::zeros();
    addWhiteNoiseAccel(Q, 0, cfg_.q_xyz, dt);
    addWhiteNoiseAccel(Q, 2, cfg_.q_xyz, dt);
    addWhiteNoiseAccel(Q, 4, cfg_.q_xyz * 0.25, dt);
    x_ = F * x_;
    P_ = F * P_ * F.t() + Q;
    stamp_ = frame.stamp;

    int best = -1;
    double best_d = 1e18;
    for (int i = 0; i < static_cast<int>(frame.armors.size()); ++i) {
        const auto& o = frame.armors[i];
        if (deviceId(o.det) != id_) continue;
        const double d = cv::norm(o.position - position(x_));
        if (d < best_d) best_d = d, best = i;
    }
    if (best >= 0 && best_d < cfg_.max_match_distance * std::max(1.0, frame.armors[best].distance / 3.0)) {
        const auto& o = frame.armors[best];
        cv::Matx<double, 3, 6> H = cv::Matx<double, 3, 6>::zeros();
        H(0, 0) = H(1, 2) = H(2, 4) = 1;
        const cv::Matx33d R = positionNoise(o, camera_.fx(), cfg_.pixel_noise);
        const cv::Vec3d y = o.position - position(x_);
        const cv::Matx33d S = H * P_ * H.t() + R;
        const cv::Matx<double, 6, 3> K = P_ * H.t() * S.inv(cv::DECOMP_CHOLESKY);
        x_ += K * y;
        P_ = (Mat::eye() - K * H) * P_;
        last_match_ = frame.stamp;
        yaw_ = o.yaw;
    } else if (best >= 0) {
        // the plate jumped (spinning target): a point predictor can only restart
        init(frame.armors[best], frame.stamp);
    } else if (frame.stamp - last_match_ > cfg_.lost_time_thres) {
        active_ = false;
        const int idx = chooseInitialTarget(frame.armors);
        if (idx >= 0) init(frame.armors[idx], frame.stamp);
    }
}

}  // namespace autoaim
