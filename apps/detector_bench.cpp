// Detector-only benchmark (no pose / tracker), mirroring the legacy sentry
// detector benchmark so the two can be compared on the same video.
#include <opencv2/opencv.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "autoaim/config.hpp"
#include "autoaim/detector.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: detector_bench <video> <out.csv> [--config f] [max_frames]\n";
        return 1;
    }
    std::string video = argv[1], out_path = argv[2], config_path = "config/default.yaml";
    int max_frames = -1;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) config_path = argv[++i];
        else max_frames = std::stoi(a);
    }
    autoaim::Config cfg;
    try {
        cfg = autoaim::Config::load(config_path);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    autoaim::Detector detector(cfg.detector);
    cv::VideoCapture cap(video);
    if (!cap.isOpened()) return 1;
    std::ofstream out(out_path);
    out << "frame,count\n";
    cv::Mat frame;
    int n = 0, frames_with = 0, total = 0;
    double ms = 0;
    while (cap.read(frame)) {
        if (max_frames > 0 && n >= max_frames) break;
        const auto t0 = std::chrono::steady_clock::now();
        const auto armors = detector.detect(frame);
        const auto t1 = std::chrono::steady_clock::now();
        ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
        out << n << ',' << armors.size() << '\n';
        frames_with += armors.empty() ? 0 : 1;
        total += static_cast<int>(armors.size());
        ++n;
    }
    std::cout << "frames=" << n << " detected_frames=" << frames_with << " armors=" << total
              << " ms_per_frame=" << (n ? ms / n : 0) << "\n";
    return 0;
}
