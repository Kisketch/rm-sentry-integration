#include "sentry/roi.hpp"

#include <cstddef>

namespace sentry {

std::vector<cv::Rect> mergeWindows(std::vector<cv::Rect> w, int slack) {
    // Repeated pairwise union until stable.  n is tiny (<= 4 slots x a few
    // robots), so the O(n^3) worst case is irrelevant.
    bool merged = true;
    while (merged) {
        merged = false;
        for (std::size_t i = 0; i < w.size() && !merged; ++i) {
            const cv::Rect grown(w[i].x - slack, w[i].y - slack, w[i].width + 2 * slack,
                                 w[i].height + 2 * slack);
            for (std::size_t j = i + 1; j < w.size(); ++j) {
                if ((grown & w[j]).area() > 0) {
                    w[i] = w[i] | w[j];
                    w.erase(w.begin() + static_cast<std::ptrdiff_t>(j));
                    merged = true;
                    break;
                }
            }
        }
    }
    return w;
}

}  // namespace sentry
