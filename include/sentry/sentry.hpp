// Sentry vision façade: the new autoaim core plus the sentry-specific pieces
// (gimbal attitude / camera extrinsic, multi-target policy, outpost model).
// Everything below this class is independent of the transport (video, HikRobot,
// serial); the app or the robot node only feeds frames and reads AimCommand.
#pragma once

#include <string>
#include <vector>

#include "autoaim/aimer.hpp"
#include "autoaim/ballistics.hpp"
#include "autoaim/config.hpp"
#include "autoaim/detector.hpp"
#include "autoaim/pose.hpp"
#include "autoaim/target.hpp"
#include "autoaim/tracker.hpp"
#include "autoaim/types.hpp"
#include "sentry/roi.hpp"
#include "sentry/serial_protocol.hpp"

namespace sentry {

struct SentryOutput {
    autoaim::FrameObservations observations;
    std::vector<autoaim::TargetState> targets;
    int selected = -1;
    autoaim::AimCommand command;
};

class SentryVision {
public:
    explicit SentryVision(const autoaim::Config& cfg);
    // Members hold references to sibling members (pose_ and manager_ ->
    // camera_, aimer_ -> ballistics_): a copy or move would dangle.
    SentryVision(const SentryVision&) = delete;
    SentryVision& operator=(const SentryVision&) = delete;

    // One frame.  `gimbal_*` is the IMU/serial attitude at the exposure time
    // (world = gimbal-centred, gravity aligned).  Pass zeros for fixed-camera
    // offline video.
    SentryOutput process(const cv::Mat& bgr, double stamp, double gimbal_yaw = 0.0,
                         double gimbal_pitch = 0.0, double gimbal_roll = 0.0);

    void setEnemyColor(autoaim::Color c) { detector_.setEnemyColor(c); }
    // Tracking-guided windowed detection (docs/roi_detection.md): search
    // only around every active track's predicted plate positions (all
    // plate slots, projected at the current frame's attitude, 60 px
    // margin), with a full scan every 30 frames and an immediate
    // full-frame retry when a windowed frame finds nothing. Off by default.
    void setRoiEnabled(bool on) { roi_enabled_ = on; }
    // Windows used for the last frame (after merging), for overlays/tests.
    const std::vector<cv::Rect>& roiWindows() const { return roi_windows_; }
    void reset() { manager_.reset(); }
    const autoaim::Config& config() const { return cfg_; }
    autoaim::Config& mutableConfig() { return cfg_; }
    const autoaim::CameraModel& camera() const { return camera_; }
    const autoaim::Detector& detector() const { return detector_; }

private:
    autoaim::Config cfg_;
    autoaim::Detector detector_;
    autoaim::CameraModel camera_;
    autoaim::PoseSolver pose_;
    autoaim::Ballistics ballistics_;
    autoaim::TargetManager manager_;
    autoaim::RobotAimer aimer_;
    bool roi_enabled_ = false;
    std::vector<cv::Rect> roi_windows_;
    int frames_since_full_ = 0;
};

// Convert a command for the old sentry 21-byte frame (AA BB ... BB) so a
// deployment can reuse the existing MCU protocol.  yaw/pitch are in radians,
// the protocol carries 1e-4 degrees, exactly like the old AutoArmor::Transmit.
SerialCommand toSerial(const autoaim::AimCommand& cmd, double compensate_yaw_deg = 0.0,
                       double compensate_pitch_deg = 0.0, double yaw_vel_rad = 0.0);


}  // namespace sentry
