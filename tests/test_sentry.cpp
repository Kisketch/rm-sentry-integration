#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "autoaim/aimer.hpp"
#include "autoaim/detector.hpp"
#include "autoaim/ballistics.hpp"
#include "autoaim/pose.hpp"
#include "autoaim/pose_buffer.hpp"
#include "autoaim/target.hpp"
#include "autoaim/tracker.hpp"
#include "sentry/roi.hpp"
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

// ---------------------------------------------------------------- added
// Regression tests for the 2026-11 review fixes (see docs/review_2026-11.md).

autoaim::ArmorObservation obsColored(const autoaim::CameraModel& cam, const cv::Vec3d& pos, double yaw,
                                     const std::string& number, autoaim::Color color) {
    auto o = makeObservation(cam, pos, yaw, autoaim::ArmorType::SMALL, 15, number);
    o.det.color = color;
    return o;
}

void testWrapAngle() {
    using autoaim::wrapAngle;
    using autoaim::kPi;
    CHECK(std::abs(wrapAngle(3 * kPi) - kPi) < 1e-12);
    CHECK(std::abs(wrapAngle(-kPi) - kPi) < 1e-12);        // (-pi, pi]
    CHECK(std::abs(wrapAngle(0.5 + 2e6 * kPi) - 0.5) < 1e-6);  // O(1) for large inputs
    // Used to spin forever: inf - 2pi == inf.
    CHECK(std::isinf(wrapAngle(std::numeric_limits<double>::infinity())));
    CHECK(std::isnan(wrapAngle(std::numeric_limits<double>::quiet_NaN())));
    for (double a = -20; a < 20; a += 0.37) {
        const double w = wrapAngle(a);
        CHECK(w > -kPi && w <= kPi);
        CHECK(std::abs(std::remainder(w - a, 2 * kPi)) < 1e-9);
    }
}

// A snapshot extrapolated twice must agree with one snapshot at the final
// time: TargetState(t1).centerAt(t2) == TargetState(t2).center.  Failed
// before the fix because predict() did not decay the Singer velocity.
void testSingerSnapshotConsistency() {
    auto cam = makeCamera();
    autoaim::TrackerConfig tc;
    tc.singer_tau = 0.3;
    autoaim::RobotTracker tracker(tc, cam);
    for (int n = 0; n < 50; ++n) {
        const double t = n * 0.04;
        autoaim::FrameObservations f;
        f.stamp = t;
        const cv::Vec3d c(5.0, -0.5 + 0.8 * t, 0.0);
        const double yaw = 0.1;
        f.armors.push_back(makeObservation(cam, {c[0] - 0.25 * std::cos(yaw), c[1] - 0.25 * std::sin(yaw), 0},
                                           yaw, autoaim::ArmorType::SMALL, 15, "3"));
        tracker.update(f);
    }
    const double t0 = 49 * 0.04, t1 = t0 + 0.05, t2 = t0 + 0.35;
    const auto s1 = tracker.targetState(t1), s2 = tracker.targetState(t2);
    CHECK(cv::norm(s1.velocity) > 0.3);  // the test needs a moving target
    CHECK(cv::norm(s1.centerAt(t2) - s2.center) < 1e-9);
    // plate height follows the centre's vertical extrapolation
    const auto a = s1.armorAt(0, t2);
    CHECK(std::abs((a.position[2] - s1.armors[0].position[2]) - (s1.centerAt(t2)[2] - s1.center[2])) < 1e-12);
}

void testTargetManagerIdentity() {
    auto cam = makeCamera();
    autoaim::TrackerConfig tc;
    tc.singer_tau = 0.3;
    autoaim::TargetConfig tg;
    tg.max_distance = 20;
    tg.require_tracking = true;

    // (a) Same number, different colours -> two robots, two tracks.
    {
        autoaim::TargetManager mgr(tc, tg, cam);
        for (int n = 0; n < 20; ++n) {
            autoaim::FrameObservations f;
            f.frame = n, f.stamp = n * 0.04;
            f.armors.push_back(obsColored(cam, {5.0, 0.3, 0.0}, 0.05, "3", autoaim::Color::RED));
            f.armors.push_back(obsColored(cam, {5.0, 0.6, 0.0}, 0.10, "3", autoaim::Color::BLUE));
            mgr.update(f);
        }
        CHECK(mgr.size() == 2);
        const auto st = mgr.snapshots(19 * 0.04);
        CHECK(st.size() == 2);
        if (st.size() == 2) CHECK(st[0].id != st[1].id);
    }
    // (b) A one-frame outlier with the same identity (a misread digit 2 m
    // away) must neither spawn a duplicate track nor knock the real one out.
    {
        autoaim::TargetManager mgr(tc, tg, cam);
        for (int n = 0; n < 30; ++n) {
            autoaim::FrameObservations f;
            f.frame = n, f.stamp = n * 0.04;
            f.armors.push_back(obsColored(cam, {5.0, 0.3, 0.0}, 0.05, "3", autoaim::Color::RED));
            if (n == 20)
                f.armors.push_back(obsColored(cam, {6.0, -1.8, 0.0}, -0.3, "3", autoaim::Color::RED));
            mgr.update(f);
            if (n >= 20) {
                CHECK(mgr.size() == 1);
                const auto st = mgr.snapshots(f.stamp);
                CHECK(st.size() == 1 && st[0].tracking());
            }
        }
    }
    // (c) The robot really is elsewhere (re-identified 2 m away and stays
    // there): the single track must re-seed and be TRACKING again quickly,
    // still with exactly one track for the identity.
    {
        autoaim::TargetManager mgr(tc, tg, cam);
        int recovered_at = -1;
        for (int n = 0; n < 40; ++n) {
            autoaim::FrameObservations f;
            f.frame = n, f.stamp = n * 0.04;
            const cv::Vec3d p = n < 20 ? cv::Vec3d(5.0, 0.3, 0.0) : cv::Vec3d(6.0, -1.8, 0.0);
            f.armors.push_back(obsColored(cam, p, n < 20 ? 0.05 : -0.3, "3", autoaim::Color::RED));
            mgr.update(f);
            CHECK(mgr.size() == 1);
            const auto st = mgr.snapshots(f.stamp);
            if (n > 20 && recovered_at < 0 && st.size() == 1 && st[0].tracking() &&
                cv::norm(st[0].armorAt(0, f.stamp).position - p) < 0.3)
                recovered_at = n;
        }
        CHECK(recovered_at > 20 && recovered_at <= 30);
    }
}

// The switch hysteresis may only keep an ELIGIBLE previous target.
void testSelectHysteresisEligibility() {
    auto cam = makeCamera();
    autoaim::TrackerConfig tc;
    autoaim::TargetConfig tg;
    tg.max_distance = 8;
    tg.require_tracking = true;
    tg.switch_margin = 0.5;
    tg.priority = {};
    autoaim::TargetManager mgr(tc, tg, cam);
    auto mk = [](const std::string& id, double x, autoaim::TrackState st) {
        autoaim::TargetState s;
        s.valid = true, s.id = id, s.label = id, s.state = st, s.armors_num = 4;
        s.center = {x, 0, 0};
        return s;
    };
    std::vector<autoaim::TargetState> st{mk("A", 5.0, autoaim::TrackState::TRACKING),
                                         mk("B", 5.2, autoaim::TrackState::TRACKING)};
    CHECK(mgr.select(st) == 0);
    // B is now slightly closer, but within the margin: keep A.
    st[1].center = {4.8, 0, 0};
    CHECK(mgr.select(st) == 0);
    // A drops to TEMP_LOST: with require_tracking it must not be kept.
    st[0].state = autoaim::TrackState::TEMP_LOST;
    CHECK(mgr.select(st) == 1);
    // Now B is current; B walks beyond max_distance, A is back: switch to A.
    st[0].state = autoaim::TrackState::TRACKING;
    st[1].center = {8.3, 0, 0};
    CHECK(mgr.select(st) == 0);
}

// Centre mode must lead the centre's translation over the flight time.
void testCenterModeLeadsTranslation() {
    autoaim::Ballistics ball({25.0, 9.8, 0.019, true});
    autoaim::AimerConfig ac;
    ac.latency = 0.06;
    ac.center_mode_omega = 12.0;
    autoaim::RobotAimer aimer(ac, ball);
    autoaim::TargetState s;
    s.valid = true, s.id = "R3:small", s.label = "3", s.state = autoaim::TrackState::TRACKING;
    s.armors_num = 4, s.stamp = 0.0, s.omega = 15.0, s.singer_tau = 0.0;
    s.center = {5.0, 0.0, 0.0};
    s.velocity = {0.0, 1.5, 0.0};  // strafing left at 1.5 m/s
    for (int k = 0; k < 4; ++k) {
        const double yaw = 0.2 + autoaim::kPi / 2 * k;
        s.armors[k].yaw = yaw, s.armors[k].radius = 0.25;
        s.armors[k].position = {5.0 - 0.25 * std::cos(yaw), -0.25 * std::sin(yaw), 0.0};
    }
    const auto cmd = aimer.aim(s, 0.0);
    CHECK(cmd.valid && cmd.center_mode);
    const cv::Vec3d ci = s.centerAt(cmd.t_impact);
    CHECK(std::abs(autoaim::wrapAngle(cmd.yaw - std::atan2(ci[1], ci[0]))) < 1e-3);
    // and clearly not the un-led direction (the old behaviour)
    const cv::Vec3d c0 = s.centerAt(ac.latency);
    CHECK(std::abs(autoaim::wrapAngle(cmd.yaw - std::atan2(c0[1], c0[0]))) > 0.03);
}

void testAimerOffsets() {
    autoaim::Ballistics ball({25.0, 9.8, 0.019, true});
    autoaim::AimerConfig a0, a1;
    a1.yaw_offset_deg = 0.5;
    a1.pitch_offset_deg = -0.5;
    autoaim::RobotAimer r0(a0, ball), r1(a1, ball);
    autoaim::TargetState s;
    s.valid = true, s.id = "R3:small", s.label = "3", s.state = autoaim::TrackState::TRACKING;
    s.armors_num = 4, s.center = {5.0, 0.4, 0.1};
    for (int k = 0; k < 4; ++k) {
        const double yaw = 0.08 + autoaim::kPi / 2 * k;
        s.armors[k].yaw = yaw, s.armors[k].radius = 0.25;
        s.armors[k].position = {5.0 - 0.25 * std::cos(yaw), 0.4 - 0.25 * std::sin(yaw), 0.1};
    }
    const auto c0 = r0.aim(s, 0.0), c1 = r1.aim(s, 0.0);
    CHECK(c0.valid && c1.valid);
    CHECK(std::abs(c1.yaw - c0.yaw - 0.5 * autoaim::kPi / 180) < 1e-9);
    CHECK(std::abs(c1.pitch - c0.pitch + 0.5 * autoaim::kPi / 180) < 1e-9);
    CHECK(c1.fire == c0.fire && c1.armor_k == c0.armor_k);
}

void testBallistics() {
    autoaim::Ballistics drag({25.0, 9.8, 0.019, true}), vac({25.0, 9.8, 0.0, false});
    for (double s = 1.0; s <= 20.0; s += 1.5) {
        for (double h : {-0.4, 0.0, 0.3, 1.2}) {
            const auto sol = drag.solve(s, h);
            CHECK(sol.ok);
            double hh = 0, t = 0;
            CHECK(drag.simulate(sol.pitch, s, hh, t));
            CHECK(std::abs(hh - h) < 5e-4);
            CHECK(std::abs(t - sol.t_fly) < 1e-9);
            // vacuum closed form vs analytic
            const auto sv = vac.solve(s, h);
            CHECK(sv.ok);
            const double vx = 25.0 * std::cos(sv.pitch), tv = s / vx;
            CHECK(std::abs(25.0 * std::sin(sv.pitch) * tv - 4.9 * tv * tv - h) < 5e-4);
            CHECK(sol.t_fly > sv.t_fly);  // drag is slower
        }
    }
    CHECK(!drag.solve(500.0, 0.0).ok);
    CHECK(!drag.solve(std::numeric_limits<double>::quiet_NaN(), 0.0).ok);
}

void testPoseBufferExtrapolation() {
    using autoaim::quatFromYawPitchRoll;
    autoaim::PoseBuffer strict(64, 0.05), loose(64, 0.05, 0.010);
    for (int i = 0; i < 10; ++i) {
        const double t = i * 0.005;  // 200 Hz, yaw rate 2 rad/s
        strict.push({t, quatFromYawPitchRoll(2.0 * t, 0.1, 0.0)});
        loose.push({t, quatFromYawPitchRoll(2.0 * t, 0.1, 0.0)});
    }
    autoaim::Quat q;
    const double newest = 9 * 0.005;
    CHECK(!strict.query(newest + 0.004, q));  // strict: never past the newest sample
    CHECK(loose.query(newest + 0.004, q));
    const autoaim::Quat truth = quatFromYawPitchRoll(2.0 * (newest + 0.004), 0.1, 0.0);
    CHECK(autoaim::quatAngleBetween(q, truth) < 1e-6);
    CHECK(!loose.query(newest + 0.011, q));  // beyond max_extrapolation
    CHECK(loose.query(0.0125, q));           // interpolation unchanged
    CHECK(autoaim::quatAngleBetween(q, quatFromYawPitchRoll(0.025, 0.1, 0.0)) < 1e-6);
}

void testMergeWindows() {
    // Four overlapping slot windows of one robot + one far window.
    std::vector<cv::Rect> w{{100, 100, 80, 60}, {150, 110, 80, 60}, {210, 100, 80, 60},
                            {120, 140, 60, 60}, {900, 500, 50, 40}};
    const auto m = sentry::mergeWindows(w);
    CHECK(m.size() == 2);
    bool found_union = false, found_far = false;
    for (const auto& r : m) {
        found_union |= r == cv::Rect(100, 100, 190, 100);
        found_far |= r == cv::Rect(900, 500, 50, 40);
    }
    CHECK(found_union && found_far);
    // Chains merge transitively even when the first pair does not touch.
    const auto c = sentry::mergeWindows({{0, 0, 10, 10}, {40, 0, 10, 10}, {8, 0, 35, 10}});
    CHECK(c.size() == 1 && c[0] == cv::Rect(0, 0, 50, 10));
    CHECK(sentry::mergeWindows({}).empty());
}

void testConfigValidation() {
    for (const char* f : {"/config/default.yaml", "/config/sentry.yaml"}) {
        bool ok = true;
        try {
            autoaim::Config::load(std::string(AUTOAIM_ROOT) + f);
        } catch (const std::exception& e) {
            std::cerr << e.what() << "\n";
            ok = false;
        }
        CHECK(ok);
    }
    autoaim::Config c;
    CHECK(c.validate().empty());
    c.ballistic.bullet_speed = 0;
    c.aimer.gimbal_tau = 0;
    c.tracker.armors_num = 5;
    c.camera.R_cam2gimbal = {1, 0, 0, 0, 1, 0, 0, 0, -1};  // det -1: a reflection
    CHECK(c.validate().size() == 4);
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
    testWrapAngle();
    testSingerSnapshotConsistency();
    testTargetManagerIdentity();
    testSelectHysteresisEligibility();
    testCenterModeLeadsTranslation();
    testAimerOffsets();
    testBallistics();
    testPoseBufferExtrapolation();
    testMergeWindows();
    testConfigValidation();
    if (failures == 0) {
        std::cout << "ALL SENTRY TESTS PASSED\n";
        return 0;
    }
    std::cout << failures << " SENTRY TEST(S) FAILED\n";
    return 1;
}
