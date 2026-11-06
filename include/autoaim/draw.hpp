// Debug overlay: detections, the robot model re-projected, aim point, shots,
// and a top-down inset of the estimated robot.
#pragma once

#include <opencv2/core.hpp>

#include "autoaim/evaluation.hpp"
#include "autoaim/pose.hpp"
#include "autoaim/tracker.hpp"

namespace autoaim {

void drawOverlay(cv::Mat& img, const FrameObservations& obs, const FrameLog& log, const RobotTracker& geom,
                 const CameraModel& camera);

}  // namespace autoaim
