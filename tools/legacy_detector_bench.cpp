// Build-only benchmark for the old sentry ArmorDetector (06_参考项目...).
// It is not part of the CMake project because it depends on the legacy header
// tree; see tools/build_legacy_bench.sh.
#include <opencv2/opencv.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "ArmorDetector.h"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: legacy_detector_bench <video> <out.csv> [max_frames]\n";
        return 1;
    }
    const std::string video = argv[1];
    const std::string out_path = argv[2];
    const int max_frames = argc > 3 ? std::stoi(argv[3]) : -1;

    ArmorDetector detector(AUTOAIM_LEGACY_ROOT "/model/mlp.onnx",
                           AUTOAIM_LEGACY_ROOT "/model/label.txt", 0.7, {"negative"});
    cv::VideoCapture cap(video);
    if (!cap.isOpened()) {
        std::cerr << "cannot open " << video << "\n";
        return 1;
    }
    std::ofstream out(out_path);
    out << "frame,count\n";
    cv::Mat frame;
    int n = 0, frames_with = 0, total = 0;
    double ms = 0;
    while (cap.read(frame)) {
        if (max_frames > 0 && n >= max_frames) break;
        const auto t0 = std::chrono::steady_clock::now();
        const auto armors = detector.Detect(frame, ENEMY_RED);
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
