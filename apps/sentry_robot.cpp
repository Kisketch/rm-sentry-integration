// Robot-side loop: camera (OpenCV index or file) + optional old serial protocol
// -> integrated sentry vision -> 21-byte control frame.  This is intentionally
// transport-thin; swap cv::VideoCapture for the HikRobot source on the IT13.
//
// Attitude timing (G-02): each frame is stamped with its estimated exposure
// time - frame receipt time minus --exposure-offset-ms for a live camera
// (bench-measured readout + transfer lag; the MVS source in rm-autoaim
// provides true hardware stamps and needs no offset) - and the gimbal
// attitude is SLERP-interpolated at that stamp by io::SerialPort.  When no
// attitude brackets the stamp the frame is skipped (never processed with a
// stale or zero attitude) and counted in the summary line.
//
//   ./build/sentry_robot --camera 0 --serial /dev/ttyACM0 --config config/sentry.yaml
//   ./build/sentry_robot --video ../video/3.avi --serial "" --render
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <iostream>
#include <string>

#include "autoaim/config.hpp"
#include "io/serial_port.hpp"
#include "sentry/sentry.hpp"
#include "sentry/serial_protocol.hpp"

namespace {

double nowSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "config/sentry.yaml";
    std::string video;
    std::string serial_device;
    int camera_index = -1;
    bool render = false;
    int max_frames = -1;
    double comp_yaw = 0, comp_pitch = 0;
    double exposure_offset_ms = 0.0;  // live camera: receipt -> exposure lag
    double stamp_delay_ms = 1.5;      // serial: MCU sample -> host receipt lag
    double max_gap_ms = 50.0;         // serial: max attitude gap to interpolate
    bool require_crc = false;         // serial: require the byte-2 CRC-8 extension

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (a == "--video" && i + 1 < argc) video = argv[++i];
        else if (a == "--camera" && i + 1 < argc) camera_index = std::stoi(argv[++i]);
        else if (a == "--serial" && i + 1 < argc) serial_device = argv[++i];
        else if (a == "--render") render = true;
        else if (a == "--max-frames" && i + 1 < argc) max_frames = std::stoi(argv[++i]);
        else if (a == "--comp-yaw" && i + 1 < argc) comp_yaw = std::stod(argv[++i]);
        else if (a == "--comp-pitch" && i + 1 < argc) comp_pitch = std::stod(argv[++i]);
        else if (a == "--exposure-offset-ms" && i + 1 < argc) exposure_offset_ms = std::stod(argv[++i]);
        else if (a == "--stamp-delay-ms" && i + 1 < argc) stamp_delay_ms = std::stod(argv[++i]);
        else if (a == "--max-gap-ms" && i + 1 < argc) max_gap_ms = std::stod(argv[++i]);
        else if (a == "--require-crc") require_crc = true;
        else {
            std::cerr << "unknown argument: " << a << "\n";
            return 1;
        }
    }

    autoaim::Config cfg;
    try {
        cfg = autoaim::Config::load(config_path);
    } catch (const std::exception& e) {
        std::cerr << "config: " << e.what() << "\n";
        return 1;
    }
    sentry::SentryVision vision(cfg);

    cv::VideoCapture cap;
    if (!video.empty()) cap.open(video);
    else if (camera_index >= 0) cap.open(camera_index, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "cannot open " << (video.empty() ? "camera" : video) << "\n";
        return 1;
    }
    if (cfg.camera.width > 0) cap.set(cv::CAP_PROP_FRAME_WIDTH, cfg.camera.width);
    if (cfg.camera.height > 0) cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.camera.height);
    cap.set(cv::CAP_PROP_FPS, cfg.eval.fps);

    std::unique_ptr<io::SerialPort> serial;
    if (!serial_device.empty())
        serial = std::make_unique<io::SerialPort>(serial_device, 115200, stamp_delay_ms / 1000.0,
                                                  max_gap_ms / 1000.0, require_crc);

    autoaim::AimCommand last_cmd;
    double last_valid = -1e9;
    double last_cmd_stamp = -1e9;
    double last_cmd_yaw = 0;
    int processed = 0;
    std::uint64_t attitude_misses = 0;
    cv::Mat frame;
    while (cap.read(frame)) {
        if (max_frames > 0 && processed >= max_frames) break;
        // Exposure-time estimate: receipt time, minus the measured lag for a
        // live camera.  Video files keep receipt-time stamps (offline mode
        // has no serial attitude to align with anyway).
        const double t_recv = nowSeconds();
        const double stamp = video.empty() ? t_recv - exposure_offset_ms / 1000.0 : t_recv;

        sentry::AttitudeFrame attitude;
        bool have_attitude = false;
        if (serial) {
            if (serial->query(stamp, attitude)) {
                have_attitude = true;
                vision.setEnemyColor(attitude.color == 1 ? autoaim::Color::RED
                                                         : autoaim::Color::BLUE);
            } else {
                ++attitude_misses;
            }
        }

        // With a serial link, a frame without a bracketed attitude is skipped
        // entirely: processing it with a stale or zero attitude would poison
        // the tracker with a wrong world frame.  The hold logic below still
        // runs, so the MCU keeps getting non-firing commands meanwhile.
        sentry::SentryOutput out;
        const bool have_out = !serial || have_attitude;
        if (have_out) {
            out = vision.process(frame, stamp, have_attitude ? attitude.yaw : 0.0,
                                 have_attitude ? attitude.pitch : 0.0,
                                 have_attitude ? attitude.roll : 0.0);
        }

        sentry::SerialCommand sc;
        if (have_out && out.command.valid) {
            double yaw_vel = 0;
            if (last_cmd_stamp > 0 && stamp > last_cmd_stamp)
                yaw_vel = autoaim::wrapAngle(out.command.yaw - last_cmd_yaw) / (stamp - last_cmd_stamp);
            yaw_vel = std::clamp(yaw_vel, -cfg.aimer.gimbal_max_rate, cfg.aimer.gimbal_max_rate);
            sc = sentry::toSerial(out.command, comp_yaw, comp_pitch, yaw_vel);
            last_cmd = out.command;
            last_cmd_yaw = out.command.yaw;
            last_cmd_stamp = stamp;
            last_valid = stamp;
        } else if (stamp - last_valid < 0.5 && last_cmd.valid) {
            // HoldFound: keep sending the last solution for 0.5 s, but never fire.
            sc = sentry::toSerial(last_cmd, comp_yaw, comp_pitch);
            sc.fire = 0;
        }
        if (serial) serial->send(sentry::packControl(sc));

        if (render && have_out) {
            for (const auto& o : out.observations.armors)
                for (int i = 0; i < 4; ++i)
                    cv::line(frame, o.det.corners[i], o.det.corners[(i + 1) % 4],
                             cv::Scalar(0, 255, 0), 2);
            if (out.command.valid) {
                const auto center = vision.camera().project(out.targets[out.selected].centerAt(stamp));
                cv::drawMarker(frame, center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 18, 2);
                cv::putText(frame,
                            cv::format("%s %s yaw %.1f pitch %.1f %s", out.targets[out.selected].label.c_str(),
                                       autoaim::toString(out.targets[out.selected].state),
                                       out.command.yaw * 180 / autoaim::kPi,
                                       out.command.pitch * 180 / autoaim::kPi,
                                       out.command.fire ? "FIRE" : ""),
                            cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                            out.command.fire ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
            }
            cv::imshow("sentry", frame);
            if (cv::waitKey(1) == 27) break;
        }
        ++processed;
    }
    std::cout << "processed " << processed << " frames";
    if (serial)
        std::cout << ", attitude misses " << attitude_misses << " (serial received "
                  << serial->receivedFrames() << ", failed queries " << serial->failedQueries()
                  << ", crc failures " << serial->crcFailures() << ")";
    std::cout << "\n";
    return 0;
}
