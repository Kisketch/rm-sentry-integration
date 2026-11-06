// Multi-target manager: keeps one tracker per plausible robot and chooses the
// aim target explicitly.  The old sentry code kept a single Aim object and
// re-initialised it whenever the first plate under 7 m changed; this keeps the
// history of every target, so switching back does not restart the filter.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autoaim/config.hpp"
#include "autoaim/tracker.hpp"
#include "autoaim/types.hpp"

namespace autoaim {

class TargetManager {
public:
    TargetManager(const TrackerConfig& tracker_cfg, const TargetConfig& target_cfg,
                  const CameraModel& camera);

    // Associate the frame's observations, update every track (including the
    // ones that were not seen this frame), and drop stale tracks.
    void update(const FrameObservations& frame);

    // Snapshots of all active tracks, extrapolated to time t.
    std::vector<TargetState> snapshots(double t) const;

    // Index into `snapshots`, or -1.  Uses TargetConfig::priority first, then
    // distance; `require_tracking` rejects DETECTING-only tracks.
    int select(const std::vector<TargetState>& states) const;

    void reset();
    std::size_t size() const { return entries_.size(); }

private:
    struct Entry {
        std::unique_ptr<RobotTracker> robot;
        std::unique_ptr<OutpostTracker> outpost;
        std::string label;
        double last_seen = 0.0;

        bool active() const;
        TrackState state() const;
        TargetState snapshot(double t) const;
        void update(const FrameObservations& frame);
        void reset();
    };

    int matchEntry(const ArmorObservation& obs, double stamp) const;

    TrackerConfig tracker_cfg_;
    TargetConfig target_cfg_;
    const CameraModel& camera_;
    std::vector<Entry> entries_;
    mutable std::string last_selected_id_;
};

}  // namespace autoaim
