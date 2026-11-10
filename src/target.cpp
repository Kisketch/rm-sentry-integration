#include "autoaim/target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoaim {

namespace {
// Matching identity of an entry: the outpost is a singleton, robots are
// keyed by deviceId so a same-number robot of another colour/size can
// never be associated into the entry (label alone allowed that).
std::string entryKey(const ArmorDetection& det) {
    return (det.number == "outpost" || det.type == ArmorType::OUTPOST) ? "outpost" : deviceId(det);
}
}  // namespace

TargetManager::TargetManager(const TrackerConfig& tracker_cfg, const TargetConfig& target_cfg,
                             const CameraModel& camera)
    : tracker_cfg_(tracker_cfg), target_cfg_(target_cfg), camera_(camera) {}

bool TargetManager::Entry::active() const {
    return robot ? robot->active() : (outpost && outpost->active());
}

TrackState TargetManager::Entry::state() const {
    return robot ? robot->state() : (outpost ? outpost->state() : TrackState::LOST);
}

void TargetManager::Entry::reset() {
    if (robot) robot->reset();
    if (outpost) outpost->reset();
}

TargetState TargetManager::Entry::snapshot(double t) const {
    if (robot) return autoaim::snapshot(*robot, t);
    if (outpost) return autoaim::snapshot(*outpost, t);
    return {};
}

void TargetManager::Entry::update(const FrameObservations& frame) {
    if (robot) robot->update(frame);
    else if (outpost) outpost->update(frame);
}

int TargetManager::matchEntry(const ArmorObservation& obs, double stamp) const {
    int best = -1;
    double best_d = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
        const Entry& e = entries_[i];
        if (e.key != entryKey(obs.det)) continue;
        const TargetState s = e.snapshot(stamp);
        const TrackState st = e.state();
        if (st == TrackState::LOST && e.last_seen <= 0) continue;
        // Range-scaled gate: a fixed 1 m gate merges two robots standing
        // within 1 m of each other at 10 m.  Close in the geometry is well
        // constrained (0.4x), far out the same gate is needed (1.0x).
        // LOST tracks get 1.4x rather than 2x: 2x was re-acquiring a target
        // into its neighbour.
        const double range_scale = std::clamp(0.35 + 0.065 * obs.distance, 0.40, 1.0);
        const double gate = target_cfg_.association_gate * range_scale *
                            (st == TrackState::LOST ? 1.4 : 1.0);
        // Prefer a TRACKING track when two candidates are close.
        const double penalty = st == TrackState::TRACKING ? 0.0 : 0.25;
        for (int k = 0; k < s.armors_num; ++k) {
            const ArmorState a = s.armorAt(k, stamp);
            const double d = cv::norm(obs.position - a.position) + penalty;
            if (d < gate && d < best_d) best_d = d, best = i;
        }
    }
    return best;
}

void TargetManager::update(const FrameObservations& frame) {
    std::vector<FrameObservations> per_entry(entries_.size());
    for (auto& f : per_entry) {
        f.frame = frame.frame;
        f.stamp = frame.stamp;
    }
    std::vector<bool> assigned(frame.armors.size(), false);

    // Pass 1: attach observations to existing tracks.
    for (int i = 0; i < static_cast<int>(frame.armors.size()); ++i) {
        const ArmorObservation& o = frame.armors[i];
        if (o.distance > target_cfg_.max_distance) continue;
        const int e = matchEntry(o, frame.stamp);
        if (e >= 0) {
            per_entry[e].armors.push_back(o);
            assigned[i] = true;
        }
    }

    // Pass 2: create tracks for unassigned observations.  Observations that are
    // close to the seed and carry the same label are attached to the new track
    // immediately, so a two-plate frame does not create two half-trackers.
    for (int i = 0; i < static_cast<int>(frame.armors.size()); ++i) {
        if (assigned[i]) continue;
        const ArmorObservation& seed = frame.armors[i];
        if (seed.distance > target_cfg_.max_distance) continue;

        // Reuse a lost track of the same label if the seed is near its last
        // position.  Without this, a tracker that drops for a few frames leaves
        // a duplicate behind and the duplicate steals observations.
        int reuse = -1;
        for (int e = 0; e < static_cast<int>(entries_.size()); ++e) {
            if (entries_[e].key != entryKey(seed.det) || entries_[e].state() != TrackState::LOST)
                continue;
            const TargetState old = entries_[e].snapshot(frame.stamp);
            for (int k = 0; k < old.armors_num; ++k) {
                if (cv::norm(seed.position - old.armorAt(k, frame.stamp).position) <
                    2.0 * target_cfg_.association_gate) {
                    reuse = e;
                    break;
                }
            }
            if (reuse >= 0) break;
        }

        FrameObservations init;
        init.frame = frame.frame;
        init.stamp = frame.stamp;
        init.armors.push_back(seed);
        assigned[i] = true;
        for (int j = i + 1; j < static_cast<int>(frame.armors.size()); ++j) {
            if (assigned[j]) continue;
            const ArmorObservation& o = frame.armors[j];
            if (entryKey(o.det) != entryKey(seed.det)) continue;
            if (cv::norm(o.position - seed.position) > 2.0 * target_cfg_.association_gate) continue;
            init.armors.push_back(o);
            assigned[j] = true;
        }

        if (reuse >= 0) {
            entries_[reuse].reset();
            entries_[reuse].last_seen = frame.stamp;
            per_entry[reuse].armors = init.armors;
            continue;
        }

        Entry e;
        e.key = entryKey(seed.det);
        e.label = seed.det.number;
        if (seed.det.number == "outpost" || seed.det.type == ArmorType::OUTPOST) {
            e.label = "outpost";
            e.outpost = std::make_unique<OutpostTracker>(tracker_cfg_, camera_);
        } else {
            e.robot = std::make_unique<RobotTracker>(tracker_cfg_, camera_);
        }
        e.last_seen = frame.stamp;
        entries_.push_back(std::move(e));
        per_entry.emplace_back();
        per_entry.back().frame = frame.frame;
        per_entry.back().stamp = frame.stamp;
        per_entry.back().armors = init.armors;
    }

    // Pass 3: update all tracks with their subset.  A newly created entry gets
    // its seed observations here, exactly once.
    for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
        const bool seen = !per_entry[i].armors.empty();
        entries_[i].update(per_entry[i]);
        if (seen) entries_[i].last_seen = frame.stamp;
    }

    // Drop stale tracks.
    entries_.erase(
        std::remove_if(entries_.begin(), entries_.end(),
                       [&](const Entry& e) {
                           const bool lost = e.robot ? e.robot->state() == TrackState::LOST
                                                     : e.outpost->state() == TrackState::LOST;
                           return lost && frame.stamp - e.last_seen > target_cfg_.lost_keep_time;
                       }),
        entries_.end());
}

std::vector<TargetState> TargetManager::snapshots(double t) const {
    std::vector<TargetState> out;
    for (const auto& e : entries_)
        if (e.state() != TrackState::LOST) out.push_back(e.snapshot(t));
    return out;
}

int TargetManager::select(const std::vector<TargetState>& states) const {
    int best = -1;
    double best_distance = std::numeric_limits<double>::max();
    int best_priority = std::numeric_limits<int>::max();
    for (int i = 0; i < static_cast<int>(states.size()); ++i) {
        const TargetState& s = states[i];
        if (!s.valid) continue;
        if (target_cfg_.require_tracking && !s.tracking()) continue;
        const double d = cv::norm(s.center);
        if (d > target_cfg_.max_distance) continue;
        int prio = std::numeric_limits<int>::max();
        for (int p = 0; p < static_cast<int>(target_cfg_.priority.size()); ++p)
            if (target_cfg_.priority[p] == s.label) {
                prio = p;
                break;
            }
        if (prio < best_priority || (prio == best_priority && d < best_distance)) {
            best_priority = prio;
            best_distance = d;
            best = i;
        }
    }
    // Target-switch hysteresis: keep the current target unless the challenger
    // is clearly closer (or has a better priority class).
    if (best >= 0 && !last_selected_id_.empty()) {
        int previous = -1;
        int previous_prio = std::numeric_limits<int>::max();
        double previous_d = 0;
        for (int i = 0; i < static_cast<int>(states.size()); ++i) {
            if (states[i].id != last_selected_id_) continue;
            previous = i;
            previous_d = cv::norm(states[i].center);
            for (int p = 0; p < static_cast<int>(target_cfg_.priority.size()); ++p)
                if (target_cfg_.priority[p] == states[i].label) {
                    previous_prio = p;
                    break;
                }
            break;
        }
        if (previous >= 0 && previous != best && previous_prio == best_priority &&
            previous_d <= best_distance + target_cfg_.switch_margin) {
            best = previous;
        }
    }
    if (best >= 0) last_selected_id_ = states[best].id;
    return best;
}

void TargetManager::reset() {
    entries_.clear();
    last_selected_id_.clear();
}

}  // namespace autoaim
