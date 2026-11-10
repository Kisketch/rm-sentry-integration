// Detection-window geometry for tracking-guided (ROI) detection.
#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace sentry {

// Merge overlapping (or nearly touching, within `slack` px) windows into their
// bounding rectangles until no two overlap.  The per-slot ROI windows of one
// robot overlap heavily; detecting in each separately repeats the mask /
// morphology / contour work on the overlap and cuts light bars at internal
// window borders (the partial contours then come back as duplicate lights).
std::vector<cv::Rect> mergeWindows(std::vector<cv::Rect> windows, int slack = 4);

}  // namespace sentry
