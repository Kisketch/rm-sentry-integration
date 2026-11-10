#include "sentry/sentry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace sentry {

using namespace autoaim;

SentryVision::SentryVision(const Config& cfg)
    : cfg_(cfg),
      detector_(cfg_.detector),
      camera_(cfg_.camera),
      pose_(camera_, cfg_.pose),
      ballistics_(cfg_.ballistic),
      manager_(cfg_.tracker, cfg_.target, camera_),
      aimer_(cfg_.aimer, ballistics_) {}

SentryOutput SentryVision::process(const cv::Mat& bgr, double stamp, double gimbal_yaw,
                                   double gimbal_pitch, double gimbal_roll) {
    camera_.setGimbalAttitude(gimbal_yaw, gimbal_pitch, gimbal_roll);

    SentryOutput out;
    out.observations.frame = static_cast<int>(std::lround(stamp * cfg_.eval.fps));
    out.observations.stamp = stamp;

    // Tracking-guided windows: build BEFORE detecting, from every active
    // track's PREDICTED plate positions at this stamp (all plate slots of
    // each tracked robot, not just the ones seen last frame), projected
    // with this frame's gimbal attitude. A plate rotating into view is
    // therefore inside a window the first frame it appears. Margin 60 px:
    // on the regression videos this beat both the previous observation-
    // based windows (misses 49 -> 3 on the slow-spin clip at equal end-to-
    // end hit rate) and a 110 px margin (bigger windows clip more light
    // bars at window borders and feed duplicate lights back to the
    // tracker); see docs/roi_detection.md.
    roi_windows_.clear();
    if (roi_enabled_) {
        constexpr int kMargin = 60;
        const cv::Rect frame_rect(0, 0, bgr.cols, bgr.rows);
        for (const auto& s : manager_.snapshots(stamp)) {
            if (!s.valid || !s.active()) continue;
            for (int k = 0; k < s.armors_num; ++k) {
                const ArmorState a = s.armorAt(k, stamp);
                // Behind or too close to the CAMERA (the gimbal x axis is not the
                // optical axis once the gimbal has turned).
                if (camera_.worldToCamera(a.position)[2] < 0.3) continue;
                const double pitch_deg = s.type == ArmorType::OUTPOST ? cfg_.pose.outpost_armor_pitch_deg
                                                                      : cfg_.pose.armor_pitch_deg;
                const auto corners = armorCorners(a.position, a.yaw, pitch_deg * kPi / 180.0, s.type);
                double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
                for (const auto& c : corners) {
                    const cv::Point2d px = camera_.project(c);
                    x0 = std::min(x0, px.x), y0 = std::min(y0, px.y);
                    x1 = std::max(x1, px.x), y1 = std::max(y1, px.y);
                }
                cv::Rect w(cvRound(x0) - kMargin, cvRound(y0) - kMargin,
                           cvRound(x1 - x0) + 2 * kMargin, cvRound(y1 - y0) + 2 * kMargin);
                w &= frame_rect;
                if (w.width >= 8 && w.height >= 8) roi_windows_.push_back(w);
            }
        }
        roi_windows_ = mergeWindows(std::move(roi_windows_));
    }

    std::vector<ArmorDetection> detections;
    if (roi_enabled_ && !roi_windows_.empty() && frames_since_full_ < 30) {
        detections = detector_.detect(bgr, roi_windows_);
        ++frames_since_full_;
        if (detections.empty()) {  // target may have left the windows: rescan now
            detections = detector_.detect(bgr);
            frames_since_full_ = 0;
        }
    } else {
        detections = detector_.detect(bgr);
        frames_since_full_ = 0;
    }
    out.observations.armors = pose_.solveAll(detections);

    manager_.update(out.observations);
    out.targets = manager_.snapshots(stamp);
    out.selected = manager_.select(out.targets);
    if (out.selected >= 0) out.command = aimer_.aim(out.targets[out.selected], stamp);
    return out;
}

SerialCommand toSerial(const AimCommand& cmd, double compensate_yaw_deg,
                       double compensate_pitch_deg, double yaw_vel_rad) {
    SerialCommand s;
    if (!cmd.valid) return s;
    constexpr double kRadToDeg = 180.0 / kPi;
    s.yaw = static_cast<int32_t>(std::lround((cmd.yaw * kRadToDeg + compensate_yaw_deg) * 1e4));
    s.pitch = static_cast<int32_t>(std::lround((cmd.pitch * kRadToDeg + compensate_pitch_deg) * 1e4));
    s.yaw_vel = static_cast<int32_t>(std::lround((yaw_vel_rad * kRadToDeg) * 1e4));
    s.found = 1;
    s.fire = cmd.fire ? 2 : 0;
    return s;
}

}  // namespace sentry
