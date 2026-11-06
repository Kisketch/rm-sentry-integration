// Old sentry pipeline benchmark (06_参考项目...): old detector -> old IPPE +
// yaw search -> old 10-state EKF -> old quintic planner.  Aims are judged with
// the same virtual-shot judge as the new pipeline, on the same future frames.
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "ArmorDetector.h"
#include "ArmorPlanner.h"
#include "ArmorSolver.h"
#include "ArmorTarget.h"

#undef bullet_speed
#undef PI

#include "autoaim/ballistics.hpp"
#include "autoaim/config.hpp"
#include "autoaim/evaluation.hpp"

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "usage: legacy_pipeline <video> <new_detections.txt> <out.csv> [config]\n";
        return 1;
    }
    const std::string video = argv[1];
    const std::string cache = argv[2];
    const std::string out_path = argv[3];
    const std::string config_path = argc > 4 ? argv[4] : "config/sentry.yaml";

    autoaim::Config cfg = autoaim::Config::load(config_path);
    // The old planner hardcodes 22 m/s and no drag (Base.h).  Judge both
    // pipelines with the same physical bullet so the comparison is fair.
    cfg.ballistic.bullet_speed = 22.0;
    cfg.ballistic.use_drag = false;
    autoaim::CameraModel camera(cfg.camera);
    autoaim::PoseSolver pose(camera, cfg.pose);
    std::vector<autoaim::FrameDetections> frames;
    if (!autoaim::loadDetections(cache, frames)) {
        std::cerr << "cannot load detections: " << cache << "\n";
        return 1;
    }
    const auto observations = autoaim::computeObservations(frames, pose);
    autoaim::Ballistics ball(cfg.ballistic);
    autoaim::ShotJudge judge(observations, camera, ball, cfg.eval.fps);

    ArmorDetector detector(AUTOAIM_LEGACY_ROOT "/model/mlp.onnx",
                           AUTOAIM_LEGACY_ROOT "/model/label.txt", 0.7, {"negative"});
    Slover solver;
    std::unique_ptr<Aim> aim;
    std::unique_ptr<Planner> planner;
    std::unique_ptr<autoaim::GimbalSim> gimbal =
        std::make_unique<autoaim::GimbalSim>(cfg.aimer.gimbal_tau, cfg.aimer.gimbal_max_rate);
    Data_Save ds;
    ds.timestamp = std::chrono::steady_clock::now();
    ds.camera_yaw_angle = ds.camera_pitch_angle = ds.camera_roll_angle = 0;
    ds.color = ENEMY_RED;
    ds.armor_or_buff = 1;

    cv::VideoCapture cap(video);
    if (!cap.isOpened()) return 1;
    std::ofstream out(out_path);
    out << "frame,shots,verifiable,hits,yaw_deg,pitch_deg\n";
    cv::Mat frame;
    int n = 0, shots = 0, verifiable = 0, hits = 0, last_target = -1000;
    const double dt = 1.0 / cfg.eval.fps;
    while (cap.read(frame)) {
        const double stamp = n * dt;
        auto armors = detector.Detect(frame, ENEMY_RED);
        solver.UpdateStatus(ds);

        // Old ChooseProcess: keep the same number; re-acquire after 1 s or when
        // the current target disappears.
        Armors selected;
        if (!aim || n - last_target > static_cast<int>(cfg.eval.fps)) {
            aim.reset();
            for (auto& a : armors) {
                solver.Solve(a);
                if (a.gim_pose.norm() < 7.0) {
                    selected.push_back(a);
                    aim = std::make_unique<Aim>(a);
                    planner = std::make_unique<Planner>();
                    last_target = n;
                    break;
                }
            }
        } else {
            for (auto& a : armors) {
                if (a.number != aim->aim_id) continue;
                solver.Solve(a);
                if (a.gim_pose.norm() < 7.0) {
                    selected.push_back(a);
                    last_target = n;
                }
                if (selected.size() > 2) break;
            }
        }
        if (!aim || selected.empty()) {
            out << n << ",0,0,0,0,0\n";
            ++n;
            continue;
        }

        aim->Resolve(selected, dt);
        auto model = aim->createModel(aim->target_state);
        const auto ga = planner->TrajectoryPlanning(model);

        autoaim::AimCommand cmd;
        cmd.valid = true;
        cmd.yaw = ga.manage_yaw;
        cmd.pitch = ga.target_pitch;
        const auto plates = model->GetArmors(0);
        double best = 1e9;
        for (const auto& p : plates) best = std::min(best, p.abs_pose.norm());
        cmd.distance = best;
        cmd.t_fly = 0.25;
        gimbal->step(cmd.yaw, cmd.pitch, dt);

        bool fired = false;
        if (autoaim::onTarget(*gimbal, cmd, autoaim::ArmorType::SMALL, cfg.aimer.fire_tolerance_scale)) {
            autoaim::ShotRecord rec;
            rec.frame = n;
            rec.result = judge.judge(stamp + cfg.aimer.latency, gimbal->yaw, gimbal->pitch,
                                     cmd.t_fly, aim->aim_id, rec);
            ++shots;
            if (rec.result != autoaim::ShotResult::UNKNOWN) ++verifiable;
            if (rec.result == autoaim::ShotResult::HIT) ++hits;
            fired = true;
        }
        out << n << ',' << (fired ? 1 : 0) << ',' << (verifiable) << ',' << hits << ','
            << ga.manage_yaw * 180 / autoaim::kPi << ',' << ga.target_pitch * 180 / autoaim::kPi
            << '\n';
        ++n;
    }
    const double fps = cfg.eval.fps;
    std::cout << "frames=" << n << " shots=" << shots << " verifiable=" << verifiable
              << " hits=" << hits << " hit_rate="
              << (verifiable ? 100.0 * hits / verifiable : 0.0) << "% hits_per_s="
              << (n ? hits * fps / n : 0.0) << "\n";
    return 0;
}
