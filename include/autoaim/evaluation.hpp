// Offline evaluation on recorded video ("virtual shooting").
//
// There is no ground truth for the real videos, but the future is recorded:
// a shot fired at frame n arrives ~0.25 s later, and the frame at the impact
// time shows where the plate really was. The ShotJudge flies the bullet with
// the simulated gimbal's angles, intersects it with every plate detected in
// the impact frame and checks whether the impact point projects inside that
// plate's panel outline. Strategies are compared on exactly the same data.
#pragma once

#include <opencv2/core.hpp>

#include <functional>
#include <string>
#include <vector>

#include "autoaim/aimer.hpp"
#include "autoaim/ballistics.hpp"
#include "autoaim/config.hpp"
#include "autoaim/pose.hpp"
#include "autoaim/tracker.hpp"
#include "autoaim/types.hpp"

namespace autoaim {

struct FrameDetections {
    int frame = 0;
    double stamp = 0;
    std::vector<ArmorDetection> armors;
};

bool saveDetections(const std::string& path, const std::vector<FrameDetections>& frames);
bool loadDetections(const std::string& path, std::vector<FrameDetections>& frames);
std::vector<FrameObservations> computeObservations(const std::vector<FrameDetections>& frames,
                                                   const PoseSolver& solver);

// Panel (hittable plate) outline in the image, from the light-bar corners.
std::vector<cv::Point2f> panelPolygon(const ArmorDetection& det);

enum class ShotResult { HIT = 1, MISS = 0, UNKNOWN = -1 };

struct ShotRecord {
    int frame = 0;
    int impact_frame = -1;
    ShotResult result = ShotResult::UNKNOWN;
    double miss_px = 0;
    cv::Point2d impact_px;
    // impact offset from the nearest panel centre, in panel half-sizes
    // (u: towards the right bar, v: towards the top of the plate)
    double off_u = 0, off_v = 0;
};

class ShotJudge {
public:
    ShotJudge(const std::vector<FrameObservations>& obs, const CameraModel& camera,
              const Ballistics& ballistics, double fps)
        : obs_(obs), camera_(camera), ball_(ballistics), fps_(fps) {}
    // id: armor number to accept ("" = any).
    ShotResult judge(double t_depart, double yaw, double pitch, double t_fly_hint, const std::string& id,
                     ShotRecord& rec) const;

    bool platesAt(double T, const std::string& id, std::vector<ArmorObservation>& out) const;

private:
    const std::vector<FrameObservations>& obs_;
    const CameraModel& camera_;
    const Ballistics& ball_;
    double fps_;
};

struct StrategyResult {
    std::string name;
    int frames = 0;
    int shots = 0;       // frames on which the strategy fired
    int verifiable = 0;  // shots whose impact frame contains a detected plate
    int hits = 0;
    double hit_rate() const { return verifiable ? double(hits) / verifiable : 0.0; }
    double hits_per_second(double fps) const { return frames ? hits * fps / frames : 0.0; }
    std::vector<ShotRecord> shots_log;
};

struct PredictionResult {
    std::string name;
    double horizon = 0;
    int samples = 0;
    double mean_px = 0, median_px = 0, p90_px = 0;
};

// Per-frame record of the robot-EKF strategy (used for rendering and CSV).
struct FrameLog {
    int frame = 0;
    double stamp = 0;
    TrackState state = TrackState::LOST;
    RobotTracker::Vec x;
    AimCommand cmd;
    double gimbal_yaw = 0, gimbal_pitch = 0;
    bool shot = false;
    ShotRecord shot_rec;
    int matched = 0;
};

struct EvalResult {
    std::vector<StrategyResult> strategies;
    std::vector<PredictionResult> predictions;
    std::vector<FrameLog> log;
    double tracking_ratio = 0;
    double median_abs_omega = 0, median_r1 = 0, median_r2 = 0, median_dz = 0;
    double frames_with_armor_ratio = 0;
};

// Custom shot judge (the simulator uses exact ground truth). Must fill rec.result.
using JudgeFn = std::function<void(double t_depart, double yaw, double pitch, double t_fly_hint,
                                   const std::string& id, ShotRecord& rec)>;

EvalResult evaluate(const Config& cfg, const std::vector<FrameObservations>& obs,
                    const CameraModel& camera, bool keep_log, const JudgeFn* judge = nullptr);

std::string formatReport(const EvalResult& r, double fps);

}  // namespace autoaim
