// Core data types shared by every stage of the pipeline.
//
// Frames used throughout:
//   camera : OpenCV optical frame. X right, Y down, Z forward (metres).
//   world  : gimbal-centred, gravity aligned. x forward, y left, z up (metres).
//            The camera is assumed to sit at the world origin (fixed camera in
//            the test videos); R_world_cam carries the camera pitch.
//
// Armor yaw follows rm_auto_aim: the armor x-axis points INTO the robot, so a
// plate seen head-on has yaw == atan2(y, x) of its own position, and the robot
// centre is  armor_position + r * (cos(yaw), sin(yaw)).
#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace autoaim {

constexpr double kPi = 3.14159265358979323846;

enum class Color { RED = 0, BLUE = 1 };
// OUTPOST is a separate plate class: the outpost carries three plates on one
// circle with three different heights.  Keeping it in the same enum lets the
// detector/pose/tracker/aimer share one pipeline without a second type system.
enum class ArmorType { SMALL = 0, LARGE = 1, OUTPOST = 2 };

inline const char* toString(ArmorType t) {
    switch (t) {
        case ArmorType::SMALL: return "small";
        case ArmorType::LARGE: return "large";
        case ArmorType::OUTPOST: return "outpost";
    }
    return "invalid";
}

// Physical armor dimensions (metres). Light bars: distance between the two bar
// centre lines x bar length. Panel: the hittable plate.
struct ArmorGeometry {
    double light_width;   // distance between bar centre lines
    double light_height;  // bar length
    double panel_width;
    double panel_height;
};
constexpr ArmorGeometry kSmallArmor{0.135, 0.055, 0.135, 0.125};
constexpr ArmorGeometry kLargeArmor{0.225, 0.055, 0.230, 0.125};
// The outpost plate is physically a large armor.  It is a separate constant so a
// team that measures a different plate can change one value without touching the
// hero/infantry geometry.
constexpr ArmorGeometry kOutpostArmor{0.225, 0.055, 0.230, 0.125};
inline const ArmorGeometry& geometryOf(ArmorType t) {
    switch (t) {
        case ArmorType::SMALL: return kSmallArmor;
        case ArmorType::LARGE: return kLargeArmor;
        case ArmorType::OUTPOST: return kOutpostArmor;
    }
    return kSmallArmor;
}

// A light bar in the image.
struct Light {
    cv::RotatedRect rect;
    cv::Point2f top, bottom, center;
    float length = 0.f;    // |top - bottom|
    float width = 0.f;     // bar thickness
    float tilt_deg = 0.f;  // angle of the long axis from vertical
    Color color = Color::RED;
};

// A matched light pair in the image (before pose estimation).
struct ArmorDetection {
    Light left, right;
    ArmorType type = ArmorType::SMALL;
    // Corner order (the contract the number classifier depends on):
    //   0 = left bottom, 1 = left top, 2 = right top, 3 = right bottom
    std::array<cv::Point2f, 4> corners;
    cv::Point2f center;
    std::string number;       // "1".."4", "outpost", "guard", "base"
    float confidence = 0.f;
    cv::Mat number_img;       // 28x28 binarised digit (debug)
    Color color = Color::RED;  // team colour the plate was detected as
};

// Identity of the physical robot a detection belongs to.  The number alone
// is NOT an identity: both teams field a "3", and a hero's large "1" is a
// different vehicle from anything else wearing "1".  Trackers and the
// target manager key on this (colour + number + plate size).
inline std::string deviceId(const ArmorDetection& det) {
    return std::string(det.color == Color::RED ? "R" : "B") + det.number +
           (det.type == ArmorType::LARGE    ? ":large"
            : det.type == ArmorType::OUTPOST ? ":outpost"
                                             : ":small");
}

// An armor with a 3-D pose in the world frame.
struct ArmorObservation {
    ArmorDetection det;
    cv::Vec3d position;      // plate centre, world frame
    double yaw = 0.0;        // world yaw of the armor x-axis (into the robot)
    double distance = 0.0;   // |position|
    double reproj_error = 0.0;  // mean corner reprojection error (px) of the final pose
    double image_center_dist = 0.0;  // px from the image centre (for target choice)
};

// Everything the pipeline produced for one frame.
struct FrameObservations {
    int frame = 0;
    double stamp = 0.0;  // seconds
    std::vector<ArmorObservation> armors;
};

// Tracker lifecycle, shared by every tracker implementation.
enum class TrackState { LOST, DETECTING, TRACKING, TEMP_LOST };
const char* toString(TrackState s);

// A tracker-independent snapshot of one target at one instant.  The aimer and
// the fire logic consume this instead of a concrete tracker, so the standard
// 4-plate robot and the 3-plate outpost use exactly the same aiming code.
struct ArmorState {
    cv::Vec3d position{0, 0, 0};
    double yaw = 0.0;      // armor x-axis, into the robot
    double radius = 0.0;   // distance from the robot centre
    double sigma_lat = 0.0;  // 1-sigma lateral position at the snapshot time
};

struct TargetState {
    bool valid = false;
    std::string id;
    std::string label;     // "3", "outpost", ...
    TrackState state = TrackState::LOST;
    int armors_num = 0;
    ArmorType type = ArmorType::SMALL;
    double stamp = 0.0;     // time the snapshot was taken
    cv::Vec3d center{0, 0, 0};
    cv::Vec3d velocity{0, 0, 0};
    double yaw = 0.0;       // robot yaw at `stamp`
    double omega = 0.0;     // rad/s
    double singer_tau = 0.0;
    std::array<ArmorState, 4> armors{};

    bool active() const { return state == TrackState::TRACKING || state == TrackState::TEMP_LOST; }
    bool tracking() const { return state == TrackState::TRACKING; }
    cv::Vec3d centerAt(double t) const;
    ArmorState armorAt(int k, double t) const;
};

// Wrap to (-pi, pi].  O(1) for any input: the previous while-loop version
// never terminated for +-inf (a diverged filter state) and took O(|a|)
// iterations for large values.  Non-finite input is returned unchanged so
// the caller's isfinite() checks still see it.
inline double wrapAngle(double a) {
    if (!std::isfinite(a)) return a;
    a = std::remainder(a, 2 * kPi);  // [-pi, pi]
    return a <= -kPi ? a + 2 * kPi : a;
}

}  // namespace autoaim
