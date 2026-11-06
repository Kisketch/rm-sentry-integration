// Aim-point selection, gimbal command and fire decision.
//
// RobotAimer (whole-robot model):
//   - fixed-point iteration over latency + time of flight (FYT2024 / rm_vision);
//   - picks the armor whose facing angle is closest to the camera, shifted
//     towards the incoming plate by side_angle when spinning (FYT2024);
//   - above center_mode_omega it aims at the robot centre and only fires when
//     some plate will face the barrel at impact time (classic anti-spin
//     strategy: the gimbal stops chasing plates and waits for them);
//   - fires only when the simulated gimbal is within the plate's angular size.
// PointAimer / DirectAimer are the baselines (point KF / no prediction).
#pragma once

#include <opencv2/core.hpp>

#include "autoaim/ballistics.hpp"
#include "autoaim/config.hpp"
#include "autoaim/tracker.hpp"
#include "autoaim/types.hpp"

namespace autoaim {

struct AimCommand {
    bool valid = false;
    bool fire = false;
    bool center_mode = false;
    double yaw = 0, pitch = 0;  // desired gimbal angles (rad)
    cv::Vec3d aim_point{0, 0, 0};
    double t_fly = 0;
    double t_impact = 0;        // absolute time
    int armor_k = -1;
    double facing_deg = 0;
    double distance = 0;
    double sigma_lat = 0;       // predicted lateral std of the aimed plate at impact (m)
};

// First-order gimbal with a rate limit - enough to make "chasing every plate"
// visibly worse than "waiting at the centre".
class GimbalSim {
public:
    GimbalSim(double tau, double max_rate, double ff_gain = 0.0)
        : tau_(tau), max_rate_(max_rate), ff_(ff_gain) {}
    void step(double cmd_yaw, double cmd_pitch, double dt);
    double yaw = 0, pitch = 0;
    bool initialised = false;

private:
    double tau_, max_rate_, ff_ = 0.0;
    double prev_yaw_ = 0, prev_pitch_ = 0;
    bool has_prev_ = false;
};

bool onTarget(const GimbalSim& g, const AimCommand& cmd, ArmorType type, double scale);

class RobotAimer {
public:
    RobotAimer(const AimerConfig& cfg, const Ballistics& ballistics) : cfg_(cfg), ball_(ballistics) {}
    // Tracker convenience overload (kept so the original video evaluator and the
    // simulator still compile unchanged).
    AimCommand aim(const RobotTracker& tracker, double now) const;
    // Tracker-independent interface used by the sentry pipeline and the outpost.
    AimCommand aim(const TargetState& target, double now) const;

private:
    int selectArmor(const TargetState& target, double t, int previous, double& facing) const;
    AimerConfig cfg_;
    const Ballistics& ball_;
    mutable int last_armor_ = -1;  // plate chosen last frame, for switch hysteresis
    mutable std::string last_id_;  // tracker id the hysteresis state belongs to
};

class PointAimer {
public:
    PointAimer(const AimerConfig& cfg, const Ballistics& ballistics) : cfg_(cfg), ball_(ballistics) {}
    AimCommand aim(const PointTracker& tracker, double now) const;

private:
    AimerConfig cfg_;
    const Ballistics& ball_;
};

class DirectAimer {
public:
    DirectAimer(const AimerConfig& cfg, const Ballistics& ballistics) : cfg_(cfg), ball_(ballistics) {}
    AimCommand aim(const FrameObservations& frame, double now) const;

private:
    AimerConfig cfg_;
    const Ballistics& ball_;
};

}  // namespace autoaim
