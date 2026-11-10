// Offline sentry runner: feed a video through the integrated pipeline, write a
// per-frame command CSV and an optional overlay.  No serial / MVS dependency.
//
//   ./build/sentry_offline ../video/3.avi --out runs/sentry3 --render
//   ./build/sentry_offline ../video/4.avi --out runs/sentry4 --config config/sentry.yaml
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "autoaim/config.hpp"
#include "sentry/sentry.hpp"

namespace {

void usage() {
    std::cout << "usage: sentry_offline <video> [--config f.yaml] [--out dir] [--render]\n"
                 "                     [--start N] [--max-frames N] [--color red|blue] [--roi]\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }
    std::string video = argv[1];
    std::string config_path = "config/sentry.yaml";
    std::string out_dir = "runs/sentry";
    bool render = false;
    int start_frame = 0;
    int max_frames = -1;
    std::string color = "red";
    bool roi = false;

    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (a == "--out" && i + 1 < argc) out_dir = argv[++i];
        else if (a == "--render") render = true;
        else if (a == "--start" && i + 1 < argc) start_frame = std::stoi(argv[++i]);
        else if (a == "--max-frames" && i + 1 < argc) max_frames = std::stoi(argv[++i]);
        else if (a == "--color" && i + 1 < argc) color = argv[++i];
        else if (a == "--roi") roi = true;  // tracking-guided windowed detection
        else {
            std::cerr << "unknown argument: " << a << "\n";
            usage();
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
    if (color == "blue") cfg.detector.enemy_color = autoaim::Color::BLUE;

    sentry::SentryVision vision(cfg);
    vision.setRoiEnabled(roi);

    cv::VideoCapture cap(video);
    if (!cap.isOpened()) {
        std::cerr << "cannot open video: " << video << "\n";
        return 1;
    }
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (!(fps > 1.0) || fps > 1000.0) fps = cfg.eval.fps;
    const int total = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::filesystem::create_directories(out_dir);
    std::ofstream csv(out_dir + "/sentry_track.csv");
    csv << "frame,stamp,targets,selected,label,state,armors,cmd_valid,fire,yaw_deg,pitch_deg,"
           "t_fly,armor_k,facing_deg,distance,sigma_lat,detections\n";
    csv << std::fixed << std::setprecision(6);

    cv::VideoWriter writer;
    if (render) {
        writer.open(out_dir + "/sentry_overlay.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                    fps, cv::Size(width, height));
    }

    if (start_frame > 0) cap.set(cv::CAP_PROP_POS_FRAMES, start_frame);
    cv::Mat frame;
    int frame_no = start_frame;
    int processed = 0;
    int detected_frames = 0;
    int locked_frames = 0;
    int fire_frames = 0;
    long long observations = 0;

    while (cap.read(frame)) {
        if (max_frames > 0 && processed >= max_frames) break;
        const double stamp = frame_no / fps;
        const auto out = vision.process(frame, stamp);

        const bool has_det = !out.observations.armors.empty();
        detected_frames += has_det ? 1 : 0;
        observations += static_cast<long long>(out.observations.armors.size());
        locked_frames += out.selected >= 0 ? 1 : 0;
        fire_frames += out.command.fire ? 1 : 0;

        std::string label, state = "-";
        int armors = 0;
        if (out.selected >= 0) {
            const auto& t = out.targets[out.selected];
            label = t.label;
            state = autoaim::toString(t.state);
            armors = t.armors_num;
        }
        const auto& c = out.command;
        csv << frame_no << ',' << stamp << ',' << out.targets.size() << ',' << out.selected << ','
            << label << ',' << state << ',' << armors << ',' << (c.valid ? 1 : 0) << ','
            << (c.fire ? 1 : 0) << ',' << (c.valid ? c.yaw * 180.0 / autoaim::kPi : 0.0) << ','
            << (c.valid ? c.pitch * 180.0 / autoaim::kPi : 0.0) << ',' << c.t_fly << ','
            << c.armor_k << ',' << c.facing_deg << ',' << c.distance << ',' << c.sigma_lat << ','
            << out.observations.armors.size() << '\n';

        if (render) {
            for (const auto& o : out.observations.armors) {
                for (int i = 0; i < 4; ++i)
                    cv::line(frame, o.det.corners[i], o.det.corners[(i + 1) % 4],
                             cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, o.det.number + " " + cv::format("%.1fm", o.distance),
                            o.det.corners[1] + cv::Point2f(0, -6), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                            cv::Scalar(0, 255, 0), 2);
            }
            if (out.selected >= 0) {
                const auto& t = out.targets[out.selected];
                const auto center = vision.camera().project(t.centerAt(stamp));
                cv::drawMarker(frame, center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 18, 2);
                if (out.command.valid && vision.camera().inFront(out.command.aim_point)) {
                    const auto aim = vision.camera().project(out.command.aim_point);
                    cv::circle(frame, aim, 6, cv::Scalar(0, 0, 255), 2);
                    cv::line(frame, center, aim, cv::Scalar(0, 0, 255), 1);
                }
                cv::putText(frame,
                            cv::format("%s %s  yaw %.1f pitch %.1f  %s", t.label.c_str(),
                                       autoaim::toString(t.state),
                                       out.command.yaw * 180.0 / autoaim::kPi,
                                       out.command.pitch * 180.0 / autoaim::kPi,
                                       out.command.fire ? "FIRE" : ""),
                            cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                            out.command.fire ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
            }
            writer.write(frame);
        }

        ++frame_no;
        ++processed;
        if (processed % 200 == 0) std::cout << "  " << processed << " frames\r" << std::flush;
    }

    std::cout << "\nframes=" << processed << " detections=" << observations
              << " detection_frames=" << detected_frames << " locked=" << locked_frames
              << " fire=" << fire_frames << " csv=" << out_dir << "/sentry_track.csv\n";
    if (render) std::cout << "overlay=" << out_dir << "/sentry_overlay.avi\n";
    (void)total;
    return 0;
}
