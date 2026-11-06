// Synthetic ground-truth simulator.
//
// A virtual robot (4 armors, two radii, two heights) spins and strafes in front
// of the configured camera. Its plates are projected into the image with pixel
// noise and random drop-outs, then fed through the SAME pose solver, trackers,
// aimers and gimbal model as the real videos. Shots are judged against the exact
// plate geometry at impact time, so the hit rates here are true hit rates and
// the tracker's state can be compared with the truth.
//
//   autoaim_sim [--config cfg.yaml] [--fps 25] [--noise 0.7] [--seconds 20]
#include <opencv2/core.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "autoaim/config.hpp"
#include "autoaim/evaluation.hpp"
#include "autoaim/pose.hpp"

using namespace autoaim;

namespace {

struct Scenario {
    std::string name;
    double x = 5.0, y = 0.0, z = -0.15;  // centre (camera at origin)
    double strafe_amp = 0, strafe_period = 2.0;
    double vx = 0;
    double omega = 0;
    // omega(t) = omega + omega_amp * sin(2 pi t / omega_period): variable-speed
    // spin, or back-and-forth swinging when omega == 0
    double omega_amp = 0, omega_period = 3.0;
    double r1 = 0.25, r2 = 0.22, dz = 0.05;
};

struct Truth {
    cv::Vec3d center;
    double yaw;
};

Truth truthAt(const Scenario& s, double t) {
    Truth tr;
    tr.center = {s.x + s.vx * t, s.y + s.strafe_amp * std::sin(2 * kPi * t / s.strafe_period), s.z};
    const double wT = 2 * kPi / s.omega_period;
    tr.yaw = 0.3 + s.omega * t + s.omega_amp / wT * (1 - std::cos(wT * t));
    return tr;
}

cv::Vec3d armorPos(const Scenario& s, const Truth& tr, int k, double& yaw) {
    yaw = tr.yaw + kPi / 2 * k;
    const double r = (k % 2) ? s.r2 : s.r1;
    const double dz = (k % 2) ? s.dz : 0.0;
    return {tr.center[0] - r * std::cos(yaw), tr.center[1] - r * std::sin(yaw), tr.center[2] + dz};
}

Light lightFrom(const cv::Point2f& top, const cv::Point2f& bottom) {
    Light l;
    l.top = top, l.bottom = bottom, l.center = (top + bottom) * 0.5f;
    l.length = static_cast<float>(cv::norm(top - bottom));
    l.width = std::max(2.f, l.length / 5.f);
    l.tilt_deg = static_cast<float>(std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) * 180 / kPi);
    l.rect = cv::RotatedRect(l.center, cv::Size2f(l.width, l.length), 0);
    return l;
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path = std::string(AUTOAIM_ROOT) + "/config/default.yaml";
    double fps = 25, noise = 0.7, seconds = 20, dropout = 0.05;
    int seed = 0;
    std::vector<std::pair<std::string, double>> sets;
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config") config_path = argv[++i];
        else if (a == "--fps") fps = std::stod(argv[++i]);
        else if (a == "--noise") noise = std::stod(argv[++i]);
        else if (a == "--seconds") seconds = std::stod(argv[++i]);
        else if (a == "--dropout") dropout = std::stod(argv[++i]);
        else if (a == "--seed") seed = std::stoi(argv[++i]);
        else if (a == "--set") {
            const std::string kv = argv[++i];
            const auto eq = kv.find('=');
            sets.emplace_back(kv.substr(0, eq), std::stod(kv.substr(eq + 1)));
        }
    }
    Config cfg = Config::load(config_path);
    for (const auto& [k, v] : sets)
        if (!setParam(cfg, k, v)) {
            std::cerr << "unknown parameter " << k << '\n';
            return 1;
        }
    cfg.eval.fps = fps;
    CameraModel camera(cfg.camera);
    PoseSolver solver(camera, cfg.pose);
    Ballistics ball(cfg.ballistic);

    std::vector<Scenario> scenarios;
    {
        Scenario s;
        s.name = "spin 2.1 rad/s at 5 m (like the videos)";
        s.omega = 2.1;
        scenarios.push_back(s);
        s.name = "spin 6 rad/s + strafe 0.5 m at 5 m";
        s.omega = 6.0, s.strafe_amp = 0.5;
        scenarios.push_back(s);
        s.name = "fast spin 10 rad/s at 4 m";
        s.omega = 10.0, s.strafe_amp = 0, s.x = 4.0;
        scenarios.push_back(s);
        s.name = "no spin, strafe 0.8 m / 2 s at 6 m";
        s.omega = 0.0, s.strafe_amp = 0.8, s.x = 6.0;
        scenarios.push_back(s);
        s.name = "spin 4 rad/s approaching 7 m -> 3 m";
        s.omega = 4.0, s.strafe_amp = 0, s.x = 7.0, s.vx = -0.2;
        scenarios.push_back(s);
        s = Scenario();
        s.name = "variable spin 0.5..4.5 rad/s at 6 m";
        s.x = 6.0, s.omega = 2.5, s.omega_amp = 2.0, s.omega_period = 3.0;
        scenarios.push_back(s);
        s = Scenario();
        s.name = "swing back and forth (+-3 rad/s, 4 s) at 6 m";
        s.x = 6.0, s.omega = 0.0, s.omega_amp = 3.0, s.omega_period = 4.0;
        scenarios.push_back(s);
        s = Scenario();
        s.name = "spin 3 rad/s + gentle strafe 0.3 m / 3 s at 6 m";
        s.x = 6.0, s.omega = 3.0, s.strafe_amp = 0.3, s.strafe_period = 3.0;
        scenarios.push_back(s);
    }

    std::printf("fps %.0f, corner noise %.2f px, dropout %.0f%%, %.0f s per scenario, fx %.0f, latency %.0f ms, "
                "bullet %.0f m/s, seed %d\n\n",
                fps, noise, dropout * 100, seconds, cfg.camera.fx, cfg.aimer.latency * 1000,
                cfg.ballistic.bullet_speed, 42 + seed);
    std::printf("| scenario | direct hit%% | point KF hit%% | robot EKF hit%% | robot hits/s | centre err cm | "
                "|w| err rad/s | radius err cm | tracking %% |\n|---|---|---|---|---|---|---|---|---|\n");

    for (const auto& sc : scenarios) {
        std::mt19937 rng(42 + static_cast<unsigned>(seed));
        std::normal_distribution<double> px_noise(0.0, noise);
        std::uniform_real_distribution<double> uni(0, 1);
        const int frames = static_cast<int>(seconds * fps);
        std::vector<FrameObservations> obs;
        for (int n = 0; n < frames; ++n) {
            const double t = n / fps;
            const Truth tr = truthAt(sc, t);
            const double los = std::atan2(tr.center[1], tr.center[0]);
            FrameDetections fd;
            fd.frame = n, fd.stamp = t;
            for (int k = 0; k < 4; ++k) {
                double yaw;
                const cv::Vec3d p = armorPos(sc, tr, k, yaw);
                if (std::abs(wrapAngle(yaw - los)) > 60 * kPi / 180) continue;  // turned away: not detected
                if (uni(rng) < dropout) continue;
                const auto c3 = armorCorners(p, yaw, cfg.pose.armor_pitch_deg * kPi / 180, ArmorType::SMALL);
                cv::Point2f c[4];
                for (int i = 0; i < 4; ++i) {
                    const cv::Point2d q = camera.project(c3[i]);
                    c[i] = cv::Point2f(static_cast<float>(q.x + px_noise(rng)), static_cast<float>(q.y + px_noise(rng)));
                }
                ArmorDetection d;
                d.left = lightFrom(c[1], c[0]);
                d.right = lightFrom(c[2], c[3]);
                d.corners = {c[0], c[1], c[2], c[3]};
                d.center = (c[0] + c[1] + c[2] + c[3]) * 0.25f;
                d.number = "3";
                d.confidence = 1.f;
                d.type = ArmorType::SMALL;
                fd.armors.push_back(d);
            }
            FrameObservations o;
            o.frame = n, o.stamp = t;
            o.armors = solver.solveAll(fd.armors);
            obs.push_back(o);
        }

        // ground-truth judge: fly the bullet, test it against the true plates
        const JudgeFn judge = [&](double t_depart, double yaw, double pitch, double t_fly_hint,
                                  const std::string&, ShotRecord& rec) {
            rec.result = ShotResult::MISS;
            for (int k = 0; k < 4; ++k) {
                double T = t_depart + t_fly_hint, ayaw = 0, h = 0, tb = 0;
                cv::Vec3d p;
                for (int it = 0; it < 3; ++it) {
                    p = armorPos(sc, truthAt(sc, T), k, ayaw);
                    if (!ball.simulate(pitch, std::hypot(p[0], p[1]), h, tb)) break;
                    T = t_depart + tb;
                }
                const Truth tr = truthAt(sc, T);
                if (std::abs(wrapAngle(ayaw - std::atan2(tr.center[1], tr.center[0]))) > kPi / 2) continue;
                const double s = std::hypot(p[0], p[1]);
                const cv::Vec3d pb(s * std::cos(yaw), s * std::sin(yaw), h);
                const cv::Vec3d local = armorRotation(ayaw, cfg.pose.armor_pitch_deg * kPi / 180).t() * (pb - p);
                if (std::abs(local[1]) < kSmallArmor.panel_width / 2 && std::abs(local[2]) < kSmallArmor.panel_height / 2) {
                    rec.result = ShotResult::HIT;
                    return;
                }
            }
        };

        const EvalResult r = evaluate(cfg, obs, camera, true, &judge);

        // state errors while TRACKING
        double ce = 0, we = 0, re = 0;
        int n_t = 0;
        for (const auto& l : r.log) {
            if (l.state != TrackState::TRACKING) continue;
            const Truth tr = truthAt(sc, l.stamp);
            ce += std::hypot(l.x[RobotTracker::XC] - tr.center[0], l.x[RobotTracker::YC] - tr.center[1]);
            const double w_true = sc.omega + sc.omega_amp * std::sin(2 * kPi * l.stamp / sc.omega_period);
            we += std::abs(l.x[RobotTracker::W] - w_true);
            // radius pair (index assignment is arbitrary)
            const double e1 = std::abs(l.x[RobotTracker::R1] - sc.r1) + std::abs(l.x[RobotTracker::R2] - sc.r2);
            const double e2 = std::abs(l.x[RobotTracker::R1] - sc.r2) + std::abs(l.x[RobotTracker::R2] - sc.r1);
            re += 0.5 * std::min(e1, e2);
            ++n_t;
        }
        if (n_t) ce /= n_t, we /= n_t, re /= n_t;
        std::printf("| %s | %.1f | %.1f | %.1f | %.2f | %.1f | %.2f | %.1f | %.0f |\n", sc.name.c_str(),
                    100 * r.strategies[0].hit_rate(), 100 * r.strategies[1].hit_rate(),
                    100 * r.strategies[2].hit_rate(), r.strategies[2].hits_per_second(fps), ce * 100, we, re * 100,
                    100 * r.tracking_ratio);
    }
    return 0;
}
