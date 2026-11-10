#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "autoaim/aimer.hpp"
#include "autoaim/detector.hpp"
#include "autoaim/ballistics.hpp"
#include "autoaim/pose.hpp"
#include "autoaim/target.hpp"
#include "autoaim/tracker.hpp"
#include "sentry/serial_protocol.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << " " #cond     \
                      << "\n";                                                   \
            ++failures;                                                          \
        }                                                                        \
    } while (0)

autoaim::CameraModel makeCamera() {
    autoaim::CameraConfig c;
    c.fx = c.fy = 1200;
    c.cx = 720;
    c.cy = 540;
    c.width = 1440;
    c.height = 1080;
    c.dist = {0, 0, 0, 0, 0};
    c.pitch_deg = 0;
    return autoaim::CameraModel(c);
}

autoaim::ArmorDetection makeDetection(const autoaim::CameraModel& cam, const cv::Vec3d& pos,
                                      double yaw, autoaim::ArmorType type, double pitch_deg,
                                      const std::string& number) {
    autoaim::ArmorDetection d;
    d.type = type;
    d.number = number;
    const double pitch = pitch_deg * autoaim::kPi / 180.0;
    const auto corners = autoaim::armorCorners(pos, yaw, pitch, type);
    for (int i = 0; i < 4; ++i) d.corners[i] = cv::Point2f(cam.project(corners[i]));
    d.center = (d.corners[0] + d.corners[1] + d.corners[2] + d.corners[3]) * 0.25f;
    d.confidence = 1.f;
    d.left.length = static_cast<float>(cv::norm(d.corners[1] - d.corners[0]));
    d.right.length = static_cast<float>(cv::norm(d.corners[2] - d.corners[3]));
    d.left.top = d.corners[1];
    d.left.bottom = d.corners[0];
    d.right.top = d.corners[2];
    d.right.bottom = d.corners[3];
    return d;
}

autoaim::ArmorObservation makeObservation(const autoaim::CameraModel& cam, const cv::Vec3d& pos,
                                          double yaw, autoaim::ArmorType type, double pitch_deg,
                                          const std::string& number) {
    autoaim::ArmorObservation o;
    o.det = makeDetection(cam, pos, yaw, type, pitch_deg, number);
    o.position = pos;
    o.yaw = yaw;
    o.distance = cv::norm(pos);
    o.image_center_dist = cv::norm(cv::Point2d(o.det.center) - cv::Point2d(720, 540));
    o.reproj_error = 0.2;
    return o;
}

void testCameraExtrinsic() {
    auto cam = makeCamera();
    cam.setGimbalAttitude(0.3, -0.1, 0.05);
    const cv::Vec3d p(3.2, 1.1, 0.4);
    const cv::Vec3d back = cam.cameraToWorld(cam.worldToCamera(p));
    CHECK(cv::norm(back - p) < 1e-9);
    CHECK(cam.inFront(p));
}

void testPoseSolver() {
    auto cam = makeCamera();
    autoaim::PoseConfig pc;
    pc.optimize_yaw = true;
    pc.armor_pitch_deg = 15;
    autoaim::PoseSolver solver(cam, pc);
    const cv::Vec3d pos(4.0, 0.8, 0.5);
    const double yaw = 0.35;
    autoaim::ArmorDetection d = makeDetection(cam, pos, yaw, autoaim::ArmorType::SMALL, 15, "3");
    autoaim::ArmorObservation o;
    CHECK(solver.solve(d, o));
    CHECK(cv::norm(o.position - pos) < 0.25);
    CHECK(std::abs(autoaim::wrapAngle(o.yaw - yaw)) < 0.12);
}

void testOutpostTracker() {
    auto cam = makeCamera();
    autoaim::TrackerConfig tc;
    tc.singer_tau = 0.3;
    tc.q_xyz = 1.0;
    tc.q_yaw = 3.0;
    tc.outpost_init_radius = 0.225;
    autoaim::OutpostTracker tracker(tc, cam);

    const double radius = 0.225;
    const cv::Vec3d center(5.0, 0.5, 0.0);
    double true_w = 2.0;
    for (int n = 0; n < 120; ++n) {
        const double t = n * 0.04;
        const double phase = 0.2 + true_w * t;
        autoaim::FrameObservations f;
        f.frame = n;
        f.stamp = t;
        for (int k = 0; k < 3; ++k) {
            const double a = phase + 2.0 * autoaim::kPi * k / 3.0;
            const cv::Vec3d p(center[0] - radius * std::cos(a),
                              center[1] - radius * std::sin(a), -0.1 + 0.1 * k);
            f.armors.push_back(makeObservation(cam, p, a, autoaim::ArmorType::OUTPOST, -15,
                                               "outpost"));
        }
        tracker.update(f);
    }
    CHECK(tracker.state() == autoaim::TrackState::TRACKING);
    CHECK(std::abs(tracker.x()[autoaim::OutpostTracker::W] - true_w) < 0.5);
    CHECK(std::abs(tracker.x()[autoaim::OutpostTracker::R] - radius) < 0.05);
}

void testTargetManager() {
    auto cam = makeCamera();
    autoaim::TrackerConfig tc;
    tc.singer_tau = 0.3;
    autoaim::TargetConfig tg;
    tg.max_distance = 20;
    tg.association_gate = 1.0;
    tg.require_tracking = false;
    tg.priority = {"4", "3"};
    autoaim::TargetManager mgr(tc, tg, cam);

    for (int n = 0; n < 40; ++n) {
        const double t = n * 0.04;
        autoaim::FrameObservations f;
        f.frame = n;
        f.stamp = t;
        f.armors.push_back(makeObservation(cam, {5.0, 1.0, 0.0}, 0.2, autoaim::ArmorType::SMALL,
                                           15, "3"));
        f.armors.push_back(makeObservation(cam, {6.0, -1.0, 0.0}, -0.3, autoaim::ArmorType::SMALL,
                                           15, "4"));
        mgr.update(f);
    }
    const auto states = mgr.snapshots(39 * 0.04);
    CHECK(states.size() == 2);
    if (states.size() == 2) {
        const int idx = mgr.select(states);
        CHECK(idx >= 0);
        if (idx >= 0) CHECK(states[idx].label == "4");
    }
}

void testAimerOnOutpost() {
    auto cam = makeCamera();
    autoaim::TrackerConfig tc;
    tc.singer_tau = 0.3;
    autoaim::OutpostTracker tracker(tc, cam);
    const double radius = 0.225;
    const cv::Vec3d center(5.0, 0.5, 0.0);
    for (int n = 0; n < 60; ++n) {
        const double t = n * 0.04;
        const double phase = 0.2 + 2.0 * t;
        autoaim::FrameObservations f;
        f.frame = n;
        f.stamp = t;
        for (int k = 0; k < 3; ++k) {
            const double a = phase + 2.0 * autoaim::kPi * k / 3.0;
            const cv::Vec3d p(center[0] - radius * std::cos(a),
                              center[1] - radius * std::sin(a), -0.1 + 0.1 * k);
            f.armors.push_back(makeObservation(cam, p, a, autoaim::ArmorType::OUTPOST, -15,
                                               "outpost"));
        }
        tracker.update(f);
    }
    autoaim::Ballistics ball({25.0, 9.8, 0.019, true});
    autoaim::AimerConfig ac;
    ac.latency = 0.06;
    autoaim::RobotAimer aimer(ac, ball);
    const auto target = tracker.targetState(60 * 0.04);
    const auto cmd = aimer.aim(target, 60 * 0.04);
    CHECK(cmd.valid);
    CHECK(cmd.t_fly > 0.05 && cmd.t_fly < 1.0);
    CHECK(cmd.armor_k >= 0 && cmd.armor_k < 3);
}

void testSerialProtocol() {
    uint8_t raw[sentry::kAttitudeSize] = {0};
    raw[0] = 0xAA;
    raw[1] = 0xBB;
    auto put = [&](int off, int32_t v) {
        raw[off] = static_cast<uint8_t>((v >> 24) & 0xff);
        raw[off + 1] = static_cast<uint8_t>((v >> 16) & 0xff);
        raw[off + 2] = static_cast<uint8_t>((v >> 8) & 0xff);
        raw[off + 3] = static_cast<uint8_t>(v & 0xff);
    };
    put(3, 123456);   // 12.3456 deg
    put(7, -20000);   // -2 deg
    put(11, 314159);  // roll raw
    raw[15] = 1;
    raw[16] = 1;
    sentry::AttitudeFrame a;
    CHECK(sentry::parseAttitude(raw, sizeof(raw), a));
    CHECK(std::abs(a.yaw - 12.3456 * autoaim::kPi / 180.0) < 1e-6);
    CHECK(std::abs(a.pitch + 2.0 * autoaim::kPi / 180.0) < 1e-6);
    CHECK(a.color == 1);

    sentry::SerialCommand c;
    c.yaw = 111;
    c.pitch = -222;
    c.yaw_vel = 333;
    c.found = 1;
    c.fire = 2;
    const auto b = sentry::packControl(c, 0);
    CHECK(b[0] == 0xAA && b[1] == 0xBB && b[20] == 0xBB);
    CHECK(b[18] == 1 && b[19] == 2);
    CHECK(b[2] == 0 && b[5] == 111);

    // CRC extension: packAttitude roundtrips (roll exercises the +pi wire
    // convention), a tampered payload fails the strict parse but still
    // passes the legacy one, and a legacy frame (byte 2 = 0) is rejected by
    // the strict parse - which is why require_crc stays opt-in.
    sentry::AttitudeFrame f;
    f.yaw = 0.3;
    f.pitch = -0.12;
    f.roll = 2.9;
    f.color = 1;
    f.armor_or_buff = 0;
    const auto packed = sentry::packAttitude(f, true);
    CHECK(packed[2] == sentry::attitudeCrc(packed.data()));
    sentry::AttitudeFrame g;
    CHECK(sentry::parseAttitudeCrc(packed.data(), packed.size(), g));
    CHECK(std::abs(g.yaw - f.yaw) < 2e-4 && std::abs(g.pitch - f.pitch) < 2e-4 &&
          std::abs(g.roll - f.roll) < 2e-4);
    CHECK(g.color == 1 && g.armor_or_buff == 0);
    auto tampered = packed;
    tampered[5] ^= 0xFF;
    sentry::AttitudeFrame t;
    CHECK(!sentry::parseAttitudeCrc(tampered.data(), tampered.size(), t));
    CHECK(sentry::parseAttitude(tampered.data(), tampered.size(), t));  // legacy ignores byte 2
    const auto legacy = sentry::packAttitude(f, false);
    CHECK(legacy[2] == 0);
    CHECK(!sentry::parseAttitudeCrc(legacy.data(), legacy.size(), t));
    CHECK(sentry::parseAttitude(legacy.data(), legacy.size(), t));
}

}  // namespace

void testDetectorDegenerateInputs() {
    // Regression: detect() used to segfault on empty/degenerate frames.
    autoaim::DetectorConfig dc;
    dc.model_path = "";
    autoaim::Detector det(dc);
    cv::Mat empty;
    CHECK(det.detect(empty).empty());
    cv::Mat tiny(1, 1, CV_8UC3, cv::Scalar(0, 0, 255));
    CHECK(det.detect(tiny).empty());
    cv::Mat gray(64, 64, CV_8UC1, cv::Scalar(200));
    CHECK(det.detect(gray).empty());
    cv::Mat fl(64, 64, CV_32FC3, cv::Scalar(0.5f, 0.5f, 0.5f));
    CHECK(det.detect(fl).empty());
    // Windowed path on a degenerate frame must be equally safe.
    CHECK(det.detect(empty, std::vector<cv::Rect>{{0, 0, 100, 100}}).empty());
}

int main() {
    testDetectorDegenerateInputs();
    testCameraExtrinsic();
    testPoseSolver();
    testOutpostTracker();
    testTargetManager();
    testAimerOnOutpost();
    testSerialProtocol();
    if (failures == 0) {
        std::cout << "ALL SENTRY TESTS PASSED\n";
        return 0;
    }
    std::cout << failures << " SENTRY TEST(S) FAILED\n";
    return 1;
}
