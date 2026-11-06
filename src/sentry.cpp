#include "sentry/sentry.hpp"

#include <cmath>

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

    const auto detections = detector_.detect(bgr);
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
