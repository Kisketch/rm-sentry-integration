#include "autoaim/aimer.hpp"

#include <algorithm>
#include <cmath>

namespace autoaim {

void GimbalSim::step(double cmd_yaw, double cmd_pitch, double dt) {
    if (!initialised) {
        yaw = cmd_yaw, pitch = cmd_pitch, initialised = true;
        prev_yaw_ = cmd_yaw, prev_pitch_ = cmd_pitch, has_prev_ = true;
        return;
    }
    // Optional velocity feedforward: the command's own rate is added to the
    // first-order lag. A real gimbal that receives yaw rate as well as yaw
    // angle behaves like this.
    double ff_yaw = 0, ff_pitch = 0;
    if (ff_ != 0.0 && has_prev_) {
        ff_yaw = ff_ * wrapAngle(cmd_yaw - prev_yaw_) / dt;
        ff_pitch = ff_ * (cmd_pitch - prev_pitch_) / dt;
    }
    prev_yaw_ = cmd_yaw, prev_pitch_ = cmd_pitch, has_prev_ = true;
    // integrate in 1 ms sub-steps
    const double h = 0.001;
    for (double t = 0; t < dt - 1e-9; t += h) {
        const double ey = wrapAngle(cmd_yaw - yaw), ep = cmd_pitch - pitch;
        const double ry = std::clamp(ey / tau_ + ff_yaw, -max_rate_, max_rate_);
        const double rp = std::clamp(ep / tau_ + ff_pitch, -max_rate_, max_rate_);
        yaw = wrapAngle(yaw + ry * h);
        pitch += rp * h;
    }
}

bool onTarget(const GimbalSim& g, const AimCommand& cmd, ArmorType type, double scale) {
    if (!cmd.valid || cmd.distance <= 0) return false;
    const ArmorGeometry& geo = geometryOf(type);
    const double tol_yaw = std::max(std::atan2(scale * geo.panel_width / 2, cmd.distance), 0.2 * kPi / 180);
    const double tol_pitch = std::max(std::atan2(scale * geo.panel_height / 2, cmd.distance), 0.2 * kPi / 180);
    return std::abs(wrapAngle(g.yaw - cmd.yaw)) < tol_yaw && std::abs(g.pitch - cmd.pitch) < tol_pitch;
}

namespace {

// Fill yaw/pitch/t_fly for a world point.  Returns false if unreachable.
bool aimAt(const Ballistics& ball, const cv::Vec3d& p, AimCommand& cmd) {
    const double s = std::hypot(p[0], p[1]);
    const BallisticSolution sol = ball.solve(s, p[2]);
    if (!sol.ok) return false;
    cmd.yaw = std::atan2(p[1], p[0]);
    cmd.pitch = sol.pitch;
    cmd.t_fly = sol.t_fly;
    cmd.aim_point = p;
    cmd.distance = cv::norm(p);
    return true;
}

double flightGuess(const Ballistics& ball, const cv::Vec3d& p) {
    const BallisticSolution s = ball.solve(std::hypot(p[0], p[1]), p[2]);
    return s.ok ? s.t_fly : cv::norm(p) / ball.config().bullet_speed;
}

}  // namespace

// ------------------------------------------------------------------ robot

int RobotAimer::selectArmor(const TargetState& target, double t, int previous,
                            double& facing) const {
    const cv::Vec3d c = target.centerAt(t);
    const double center_dir = std::atan2(c[1], c[0]);
    const double w = target.omega;
    double shift = 0;
    if (std::abs(w) > cfg_.min_switch_omega) shift = (w > 0 ? 1 : -1) * cfg_.side_angle_deg * kPi / 180;
    int best = 0;
    double best_cost = 1e9;
    double cost[8] = {1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9};
    double alpha[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    const int n = std::min(8, target.armors_num);
    for (int k = 0; k < n; ++k) {
        const ArmorState a = target.armorAt(k, t);
        const double rel = wrapAngle(a.yaw - center_dir);
        alpha[k] = rel;
        cost[k] = std::abs(wrapAngle(rel + shift));
        if (cost[k] < best_cost) best_cost = cost[k], best = k;
    }
    // Hysteresis: staying on the current plate avoids dithering at the switch
    // boundary, which costs gimbal travel and therefore shots. `previous` is the
    // plate committed on the last frame, not the intermediate pick of an earlier
    // flight-time iteration (those are not new selections).
    if (cfg_.switch_hysteresis_deg > 0 && previous >= 0 && previous < n) {
        const double margin = cfg_.switch_hysteresis_deg * kPi / 180;
        if (cost[previous] <= best_cost + margin) best = previous;
    }
    facing = alpha[best];
    return best;
}

AimCommand RobotAimer::aim(const RobotTracker& tracker, double now) const {
    if (tracker.state() == TrackState::LOST) {
        last_armor_ = -1;
        last_id_.clear();
        return {};
    }
    return aim(tracker.targetState(now), now);
}

AimCommand RobotAimer::aim(const TargetState& target, double now) const {
    AimCommand cmd;
    if (target.state == TrackState::LOST) {
        last_armor_ = -1;
        last_id_.clear();
        return cmd;
    }
    if (last_id_ != target.id) {  // new target: hysteresis state is meaningless
        last_armor_ = -1;
        last_id_ = target.id;
    }
    const int previous = last_armor_;
    const double t0 = now + cfg_.latency;
    const double t_center = t0;
    const cv::Vec3d c0 = target.centerAt(t_center);
    double t_fly = flightGuess(ball_, c0);
    const bool center_mode = std::abs(target.omega) > cfg_.center_mode_omega;
    int k = 0;
    double facing = 0;
    for (int it = 0; it < 4; ++it) {
        k = selectArmor(target, t0 + t_fly, previous, facing);
        const ArmorState a = target.armorAt(k, t0 + t_fly);
        cv::Vec3d aim_point;
        if (center_mode) {
            // aim along the centre direction at the distance of the front plate
            const double r = a.radius;
            const double dc = std::hypot(c0[0], c0[1]);
            const double s = std::max(0.3, dc - r);
            aim_point = cv::Vec3d(c0[0] / dc * s, c0[1] / dc * s, a.position[2]);
        } else {
            aim_point = a.position;
        }
        AimCommand tmp;
        if (!aimAt(ball_, aim_point, tmp)) return cmd;
        const double new_t = tmp.t_fly;
        cmd = tmp;
        cmd.armor_k = k;
        cmd.facing_deg = facing * 180 / kPi;
        cmd.center_mode = center_mode;
        cmd.t_impact = t0 + cmd.t_fly;
        cmd.sigma_lat = a.sigma_lat;
        if (std::abs(new_t - t_fly) < 1e-4) break;
        t_fly = new_t;
    }
    cmd.valid = true;
    last_armor_ = k;  // committed choice for the next frame's hysteresis

    const ArmorGeometry& geo = geometryOf(target.type);
    const bool confident =
        cfg_.fire_sigma_scale <= 0 || cmd.sigma_lat < cfg_.fire_sigma_scale * geo.panel_width / 2;
    const bool tracking = target.tracking();
    if (center_mode) {
        // the plate passes the aim line; it is hittable while its centre is within
        // the tolerance, i.e. while |r sin(facing)| < tol
        const double r = target.armorAt(k, t0 + cmd.t_fly).radius;
        const double tol = cfg_.fire_tolerance_scale * geo.panel_width / 2;
        const double window =
            std::min(cfg_.center_fire_window_deg, std::asin(std::min(1.0, tol / std::max(r, 1e-3))) * 180 / kPi);
        cmd.fire = tracking && confident && std::abs(cmd.facing_deg) < window;
    } else {
        cmd.fire = tracking && confident && std::abs(cmd.facing_deg) < cfg_.max_fire_facing_deg;
    }
    return cmd;
}

// ------------------------------------------------------------------ point KF

AimCommand PointAimer::aim(const PointTracker& tracker, double now) const {
    AimCommand cmd;
    if (!tracker.active()) return cmd;
    const double t0 = now + cfg_.latency;
    double t_fly = flightGuess(ball_, tracker.position(tracker.predict(t0)));
    for (int it = 0; it < 4; ++it) {
        AimCommand tmp;
        if (!aimAt(ball_, tracker.position(tracker.predict(t0 + t_fly)), tmp)) return cmd;
        cmd = tmp;
        if (std::abs(tmp.t_fly - t_fly) < 1e-4) break;
        t_fly = tmp.t_fly;
    }
    cmd.valid = true;
    cmd.t_impact = t0 + cmd.t_fly;
    cmd.fire = true;
    return cmd;
}

// ------------------------------------------------------------------ direct

AimCommand DirectAimer::aim(const FrameObservations& frame, double now) const {
    AimCommand cmd;
    const int idx = chooseInitialTarget(frame.armors);
    if (idx < 0) return cmd;
    if (!aimAt(ball_, frame.armors[idx].position, cmd)) return cmd;
    cmd.valid = true;
    cmd.t_impact = now + cfg_.latency + cmd.t_fly;
    cmd.fire = true;
    return cmd;
}

}  // namespace autoaim
