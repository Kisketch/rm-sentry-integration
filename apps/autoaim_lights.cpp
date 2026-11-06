// Light-blob visualisation: what the detector actually sees.
//
//   autoaim_lights <video> [--frames 0,875,1500] [--out docs/detections]
//                  [--clip A:B] [--scale 0.5] [--config cfg.yaml] [--set k=v]
//
// For every requested frame it writes a 3-panel PNG:
//   left   - the frame with accepted light bars (yellow), armour boxes (green)
//            and the class-difference mask tinted red
//   middle - the binarised channel-difference mask the detector works on
//   right  - the raw R-B (red) / B-R (blue) difference, normalised
//
// --clip A:B writes <stem>_lights.mp4 (25 fps) with the same overlay, for the
// README demo.
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "autoaim/config.hpp"
#include "autoaim/detector.hpp"

using namespace autoaim;

namespace {

cv::Mat toBgr(const cv::Mat& gray) {
    cv::Mat out;
    cv::cvtColor(gray, out, cv::COLOR_GRAY2BGR);
    return out;
}

// The mask is single-channel; tint it red over the frame.
cv::Mat tintMask(const cv::Mat& frame, const cv::Mat& mask) {
    cv::Mat red(frame.size(), frame.type(), cv::Scalar(0, 0, 0));
    red.setTo(cv::Scalar(0, 0, 255), mask);
    cv::Mat out;
    cv::addWeighted(frame, 1.0, red, 0.35, 0, out);
    return out;
}

void drawLights(cv::Mat& img, const std::vector<Light>& lights) {
    for (const auto& l : lights) {
        cv::Point2f p[4];
        l.rect.points(p);
        for (int i = 0; i < 4; ++i) cv::line(img, p[i], p[(i + 1) % 4], cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        cv::circle(img, l.center, 2, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
    }
}

void drawArmors(cv::Mat& img, const std::vector<ArmorDetection>& armors, bool with_text = true) {
    for (const auto& a : armors) {
        std::vector<cv::Point> poly;
        for (const auto& c : a.corners) poly.emplace_back(cvRound(c.x), cvRound(c.y));
        cv::polylines(img, poly, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        if (!with_text) continue;
        const bool has_number = !a.number.empty() && a.number != "unknown";
        const std::string label =
            has_number ? a.number + " " + cv::format("%.2f", a.confidence)
                       : (a.type == ArmorType::SMALL ? "small" : "large");
        cv::putText(img, label, poly[1] + cv::Point(2, -6), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
}

cv::Mat stackPanels(const cv::Mat& a, const cv::Mat& b, const cv::Mat& c) {
    cv::Mat h1, h2, out;
    cv::hconcat(a, b, h1);
    cv::hconcat(h1, c, h2);
    cv::copyMakeBorder(h2, out, 8, 8, 8, 8, cv::BORDER_CONSTANT, cv::Scalar(30, 30, 30));
    return out;
}

void label(cv::Mat& img, const std::string& s) {
    cv::putText(img, s, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(img, s, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    std::string video, out_dir = "docs/detections";
    std::string config_path = std::string(AUTOAIM_ROOT) + "/config/default.yaml";
    std::vector<int> frames;
    int clip_a = -1, clip_b = -1;
    double scale = 0.5;
    std::vector<std::pair<std::string, double>> sets;

    std::string path;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--out" && i + 1 < argc) out_dir = argv[++i];
        else if (a == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (a == "--scale" && i + 1 < argc) scale = std::stod(argv[++i]);
        else if (a == "--frames" && i + 1 < argc) {
            std::string list = argv[++i];
            std::size_t pos = 0;
            while (pos <= list.size()) {
                const auto comma = list.find(',', pos);
                const std::string tok = list.substr(pos, comma - pos);
                if (!tok.empty()) frames.push_back(std::stoi(tok));
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
        } else if (a == "--clip" && i + 1 < argc) {
            std::sscanf(argv[++i], "%d:%d", &clip_a, &clip_b);
        } else if (a == "--set" && i + 1 < argc) {
            const std::string kv = argv[++i];
            const auto eq = kv.find('=');
            sets.emplace_back(kv.substr(0, eq), std::stod(kv.substr(eq + 1)));
        } else if (a == "--config") {
            config_path = argv[++i];
        } else {
            path = a;
        }
    }
    if (path.empty()) {
        std::fprintf(stderr,
                     "usage: %s <video> [--frames 0,875] [--out dir] [--clip A:B] [--scale 0.5]\n"
                     "          [--config cfg.yaml] [--set key=value]\n",
                     argv[0]);
        return 1;
    }

    Config cfg = Config::load(config_path);
    for (const auto& [k, v] : sets)
        if (!setParam(cfg, k, v)) {
            std::fprintf(stderr, "unknown parameter %s\n", k.c_str());
            return 1;
        }
    Detector detector(cfg.detector);

    cv::VideoCapture cap(path);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "cannot open %s\n", path.c_str());
        return 1;
    }
    const std::string stem = std::filesystem::path(path).stem().string();
    cv::Mat frame;
    int saved = 0;

    for (int f : frames) {
        cap.set(cv::CAP_PROP_POS_FRAMES, f);
        if (!cap.read(frame)) {
            std::fprintf(stderr, "cannot read frame %d\n", f);
            continue;
        }
        DetectorDebug dbg;
        const auto armors = detector.detect(frame, &dbg);

        cv::Mat diff, mask, panel_a, panel_b, panel_c;
        cv::Mat ch[3];
        cv::split(frame, ch);
        cv::Mat d = cfg.detector.enemy_color == Color::RED ? ch[2] - ch[0] : ch[0] - ch[2];
        // Raw difference is dominated by the saturated bar centres; a fixed x3
        // gain shows the dim rims without clipping the whole image to black.
        d.convertTo(diff, CV_8U, 3.0);

        cv::Mat overlay = tintMask(frame, dbg.mask);
        drawLights(overlay, dbg.lights);
        drawArmors(overlay, armors);

        panel_a = overlay.clone();
        if (!armors.empty()) {
            cv::Rect box = cv::boundingRect(std::vector<cv::Point>{
                {cvRound(armors[0].corners[0].x), cvRound(armors[0].corners[0].y)}});
            for (const auto& a : armors)
                for (const auto& c : a.corners) {
                    box |= cv::Rect(cvRound(c.x), cvRound(c.y), 1, 1);
                }
            for (const auto& l : dbg.lights) {
                cv::Point2f p[4];
                l.rect.points(p);
                for (const auto& q : p) box |= cv::Rect(cvRound(q.x), cvRound(q.y), 1, 1);
            }
            const int pad = std::max(20, box.width / 2);
            box = cv::Rect(box.x - pad, box.y - pad, box.width + 2 * pad, box.height + 2 * pad) &
                  cv::Rect(0, 0, frame.cols, frame.rows);
            if (box.area() > 0) {
                cv::Mat inset;
                cv::resize(overlay(box), inset, cv::Size(), 2.5, 2.5, cv::INTER_LINEAR);
                const int w = std::min(inset.cols, panel_a.cols - 10);
                const int h = std::min(inset.rows, panel_a.rows / 2);
                if (w > 20 && h > 20) {
                    cv::Mat clipped;
                    cv::resize(inset, clipped, cv::Size(w, h));
                    clipped.copyTo(panel_a(cv::Rect(panel_a.cols - w - 5, 40, w, h)));
                    cv::rectangle(panel_a, cv::Rect(panel_a.cols - w - 5, 40, w, h),
                                  cv::Scalar(255, 255, 255), 1);
                }
            }
        }
        label(panel_a, "detections  lights=" + std::to_string(dbg.lights.size()) +
                           "  armors=" + std::to_string(armors.size()));
        panel_b = toBgr(dbg.mask);
        label(panel_b, "binarised difference (threshold " +
                           std::to_string(cfg.detector.channel_diff_threshold) + ")");
        panel_c = toBgr(diff);
        label(panel_c, "R-B raw x3");

        if (scale != 1.0) {
            const cv::Size s(cvRound(panel_a.cols * scale), cvRound(panel_a.rows * scale));
            cv::resize(panel_a, panel_a, s, 0, 0, cv::INTER_AREA);
            cv::resize(panel_b, panel_b, s, 0, 0, cv::INTER_AREA);
            cv::resize(panel_c, panel_c, s, 0, 0, cv::INTER_AREA);
        }
        const cv::Mat sheet = stackPanels(panel_a, panel_b, panel_c);
        std::filesystem::create_directories(out_dir);
        char name[512];
        std::snprintf(name, sizeof(name), "%s/%s_f%05d.png", out_dir.c_str(), stem.c_str(), f);
        cv::imwrite(name, sheet);
        ++saved;
        std::printf("frame %d: %zu lights, %zu armors -> %s\n", f, dbg.lights.size(), armors.size(),
                    name);
    }

    if (clip_a >= 0 && clip_b > clip_a) {
        const double fps = cap.get(cv::CAP_PROP_FPS) > 0 ? cap.get(cv::CAP_PROP_FPS) : 25.0;
        cv::VideoWriter writer;
        const std::string mp4 = out_dir + "/" + stem + "_lights.avi";
        cap.set(cv::CAP_PROP_POS_FRAMES, clip_a);
        int n = clip_a;
        for (; n <= clip_b && cap.read(frame); ++n) {
            DetectorDebug dbg;
            const auto armors = detector.detect(frame, &dbg);
            cv::Mat img = tintMask(frame, dbg.mask);
            drawLights(img, dbg.lights);
            drawArmors(img, armors);
            label(img, stem + "  frame " + std::to_string(n) + "  lights " +
                           std::to_string(dbg.lights.size()) + "  armors " +
                           std::to_string(armors.size()));
            if (scale != 1.0)
                cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_AREA);
            if (!writer.isOpened()) {
                writer.open(mp4, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, img.size());
                if (!writer.isOpened()) {
                    std::fprintf(stderr, "cannot open video writer %s\n", mp4.c_str());
                    return 1;
                }
            }
            writer.write(img);
        }
        writer.release();
        std::printf("clip %d..%d -> %s\n", clip_a, n - 1, mp4.c_str());
    }

    std::printf("wrote %d panel images to %s\n", saved, out_dir.c_str());
    return saved > 0 || (clip_a >= 0) ? 0 : 1;
}

