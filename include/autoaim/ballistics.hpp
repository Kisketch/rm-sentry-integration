// Projectile model and pitch solving.
//
// rmoss_core / FYT2024 use closed-form approximations (pure gravity, or the
// one-dimensional "x = ln(1 + k v t) / k" drag model). Here the trajectory is
// integrated numerically (RK4) with full quadratic drag  a = -g z - k |v| v,
// which stays accurate for steep shots and long ranges; pitch is solved by a
// fixed-point iteration on the aim height (same scheme as FYT2024).
#pragma once

#include "autoaim/config.hpp"

namespace autoaim {

struct BallisticSolution {
    bool ok = false;
    double pitch = 0;  // rad, positive up
    double t_fly = 0;  // s
};

class Ballistics {
public:
    explicit Ballistics(const BallisticConfig& cfg);
    // Height and time of flight when the bullet reaches horizontal range s.
    bool simulate(double pitch, double s, double& height, double& t) const;
    // Pitch that hits (s, h); s horizontal range, h height (metres).
    BallisticSolution solve(double s, double h) const;
    const BallisticConfig& config() const { return cfg_; }

private:
    BallisticConfig cfg_;
};

}  // namespace autoaim
