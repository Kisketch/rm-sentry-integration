#include "autoaim/target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

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

void TargetManager::update(const FrameObservations& frame) {
    // One track per identity.  A deviceId (colour + number + plate size) names
    // exactly one physical robot and the outpost is a singleton, so every
    // observation of a key is routed to that key's tracker.  The tracker's own
    // association (Mahalanobis gate per plate) rejects outliers such as a
    // misread digit, and its divergence guard re-seeds the filter when the
    // robot is clearly elsewhere.
    //
    // The previous version pre-gated observations spatially and spawned a
    // second entry for the same key when the gate failed.  That produced
    // duplicate tracks sharing one id (the selector's and the aimer's
    // hysteresis keyed on that id could then flip between them) and hid the
    // far observation from the original tracker, which disabled its
    // divergence guard.
    std::vector<std::string> keys;  // first-seen order keeps updates deterministic
    std::map<std::string, std::vector<ArmorObservation>> by_key;
    for (const ArmorObservation& o : frame.armors) {
        if (!(o.distance <= target_cfg_.max_distance)) continue;  // also drops NaN
        const std::string k = entryKey(o.det);
        auto it = by_key.find(k);
        if (it == by_key.end()) {
            keys.push_back(k);
            it = by_key.emplace(k, std::vector<ArmorObservation>{}).first;
        }
        it->second.push_back(o);
    }

    for (const std::string& k : keys) {
        const bool exists = std::any_of(entries_.begin(), entries_.end(),
                                        [&k](const Entry& e) { return e.key == k; });
        if (exists) continue;
        const ArmorObservation& seed = by_key[k].front();
        Entry e;
        e.key = k;
        if (k == "outpost") {
            e.label = "outpost";
            e.outpost = std::make_unique<OutpostTracker>(tracker_cfg_, camera_);
        } else {
            e.label = seed.det.number;
            e.robot = std::make_unique<RobotTracker>(tracker_cfg_, camera_);
        }
        e.last_seen = frame.stamp;
        entries_.push_back(std::move(e));
    }

    // Every track is updated, seen or not: an unseen one predicts forward and
    // walks its own TRACKING -> TEMP_LOST -> LOST state machine.
    for (Entry& e : entries_) {
        FrameObservations sub;
        sub.frame = frame.frame;
        sub.stamp = frame.stamp;
        auto it = by_key.find(e.key);
        if (it != by_key.end()) sub.armors = std::move(it->second);
        const bool seen = !sub.armors.empty();
        e.update(sub);
        if (seen) e.last_seen = frame.stamp;
    }

    // Drop stale tracks.
    entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                                  [&](const Entry& e) {
                                      return e.state() == TrackState::LOST &&
                                             frame.stamp - e.last_seen > target_cfg_.lost_keep_time;
                                  }),
                   entries_.end());
}

std::vector<TargetState> TargetManager::snapshots(double t) const {
    std::vector<TargetState> out;
    for (const auto& e : entries_)
        if (e.state() != TrackState::LOST) out.push_back(e.snapshot(t));
    return out;
}

int TargetManager::priorityOf(const std::string& label) const {
    for (int p = 0; p < static_cast<int>(target_cfg_.priority.size()); ++p)
        if (target_cfg_.priority[p] == label) return p;
    return std::numeric_limits<int>::max();
}

bool TargetManager::eligible(const TargetState& s) const {
    if (!s.valid) return false;
    if (target_cfg_.require_tracking && !s.tracking()) return false;
    return cv::norm(s.center) <= target_cfg_.max_distance;
}

int TargetManager::select(const std::vector<TargetState>& states) const {
    int best = -1;
    double best_distance = std::numeric_limits<double>::max();
    int best_priority = std::numeric_limits<int>::max();
    int previous = -1;
    for (int i = 0; i < static_cast<int>(states.size()); ++i) {
        const TargetState& s = states[i];
        if (!eligible(s)) continue;
        if (!last_selected_id_.empty() && s.id == last_selected_id_) previous = i;
        const double d = cv::norm(s.center);
        const int prio = priorityOf(s.label);
        if (prio < best_priority || (prio == best_priority && d < best_distance)) {
            best_priority = prio;
            best_distance = d;
            best = i;
        }
    }
    // Target-switch hysteresis: keep the current target unless the challenger
    // has a better priority class or is clearly closer.  Only an ELIGIBLE
    // previous target can be kept: the old code also revived one that had
    // dropped to TEMP_LOST/DETECTING (with require_tracking) or had moved
    // beyond max_distance, because it searched all states for the id.
    if (best >= 0 && previous >= 0 && previous != best &&
        priorityOf(states[previous].label) == best_priority &&
        cv::norm(states[previous].center) <= best_distance + target_cfg_.switch_margin) {
        best = previous;
    }
    if (best >= 0) last_selected_id_ = states[best].id;
    return best;
}

void TargetManager::reset() {
    entries_.clear();
    last_selected_id_.clear();
}

}  // namespace autoaim
