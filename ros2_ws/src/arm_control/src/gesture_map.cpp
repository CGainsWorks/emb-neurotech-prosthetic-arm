#include "arm_control/gesture_map.hpp"

#include <algorithm>

namespace arm_control {
namespace {

// Joint order: shoulder rotation, shoulder flexion, elbow flexion, forearm
// rotation, wrist flexion, tool roll.
GesturePose make(uint8_t id, const char* name, ak::JointVector pose, bool moves_arm = true) {
    GesturePose entry;
    entry.id = id;
    entry.name = name;
    entry.pose = pose;
    entry.moves_arm = moves_arm;
    return entry;
}

// The pose the arm holds when nothing is being asked of it: elbow bent, hand
// in front of the body, which is where a wearer would rest an arm rather than
// straight down where it would swing.
constexpr ak::JointVector kRest{{0.0, 0.6, -1.1, 0.0, 0.3, 0.0}};

ak::JointVector from_rest(double forearm, double wrist, double tool) {
    ak::JointVector pose = kRest;
    pose[3] = forearm;
    pose[4] = wrist;
    pose[5] = tool;
    return pose;
}

}  // namespace

std::vector<GesturePose> default_gesture_poses() {
    return {
        make(0, "rest", kRest),

        // Hand configurations. The arm holds its resting posture and only the
        // terminal device roll changes, because this arm has no fingers to
        // form them with.
        make(1, "thumb up", from_rest(0.0, 0.3, 0.0), false),
        make(2, "index and middle extended, others flexed", from_rest(0.0, 0.3, 0.0), false),
        make(3, "ring and little flexed, thumb over fingers", from_rest(0.0, 0.3, 0.0), false),
        make(4, "thumb opposing little finger base", from_rest(0.0, 0.3, 0.0), false),
        make(5, "all fingers abducted", from_rest(0.0, 0.3, 0.0), false),
        make(6, "fingers flexed together in fist", from_rest(0.0, 0.3, 0.0), false),
        make(7, "pointing index", from_rest(0.0, 0.1, 0.0), false),
        make(8, "adduction of extended fingers", from_rest(0.0, 0.3, 0.0), false),

        // Wrist movements. These the arm can do and they are what the three
        // distal joints are for.
        make(9, "wrist supination (middle finger axis)", from_rest(1.2, 0.3, 0.0)),
        make(10, "wrist pronation (middle finger axis)", from_rest(-1.2, 0.3, 0.0)),
        make(11, "wrist supination (little finger axis)", from_rest(1.6, 0.3, 0.4)),
        make(12, "wrist pronation (little finger axis)", from_rest(-1.6, 0.3, -0.4)),
        make(13, "wrist flexion", from_rest(0.0, 1.1, 0.0)),
        make(14, "wrist extension", from_rest(0.0, -0.9, 0.0)),

        // Radial and ulnar deviation are the two the wrist cannot do: the
        // three wrist axes were made concurrent to get a closed-form inverse
        // solution and deviation was what that cost. They map onto tool roll,
        // which is the nearest thing the mechanism has and are marked as not
        // moving the arm so nobody mistakes the substitution for the movement.
        make(15, "wrist radial deviation", from_rest(0.0, 0.3, 0.5), false),
        make(16, "wrist ulnar deviation", from_rest(0.0, 0.3, -0.5), false),

        make(17, "wrist extension with closed hand", from_rest(0.0, -0.9, 0.0)),
    };
}

const char* outcome_name(MapOutcome outcome) {
    switch (outcome) {
        case MapOutcome::kNoChange: return "no change";
        case MapOutcome::kAccepted: return "accepted";
        case MapOutcome::kBelowThreshold: return "below confidence threshold";
        case MapOutcome::kInvalid: return "classifier reported no decision";
        case MapOutcome::kStale: return "no decision recently enough";
        case MapOutcome::kUnknownGesture: return "unknown gesture id";
        case MapOutcome::kDwelling: return "held, waiting out the dwell";
    }
    return "unknown";
}

GestureMapper::GestureMapper(std::vector<GesturePose> poses, const GestureMapConfig& config)
    : poses_(std::move(poses)), config_(config) {
    const GesturePose* rest = pose_for(0);
    accepted_pose_ = rest != nullptr ? rest->pose : ak::JointVector{};
}

const GesturePose* GestureMapper::pose_for(uint8_t gesture_id) const {
    const auto it = std::find_if(poses_.begin(), poses_.end(),
                                 [gesture_id](const GesturePose& p) { return p.id == gesture_id; });
    return it == poses_.end() ? nullptr : &*it;
}

MapResult GestureMapper::update(uint8_t gesture_id, double confidence, bool valid, double now_s) {
    MapResult result;
    result.accepted_gesture = accepted_;
    result.target = accepted_pose_;

    last_decision_s_ = now_s;
    started_ = true;

    if (!valid) {
        // The classifier could not decide, which is different from deciding
        // badly. The candidate is abandoned rather than credited with the time.
        candidate_ = accepted_;
        candidate_since_s_ = now_s;
        result.outcome = MapOutcome::kInvalid;
        return result;
    }

    const GesturePose* pose = pose_for(gesture_id);
    if (pose == nullptr) {
        result.outcome = MapOutcome::kUnknownGesture;
        return result;
    }

    const bool is_rest = gesture_id == 0;
    if (confidence < config_.confidence_threshold && !(is_rest && config_.rest_is_immediate)) {
        candidate_ = accepted_;
        candidate_since_s_ = now_s;
        result.outcome = MapOutcome::kBelowThreshold;
        return result;
    }

    if (gesture_id == accepted_) {
        candidate_ = accepted_;
        candidate_since_s_ = now_s;
        result.outcome = MapOutcome::kNoChange;
        return result;
    }

    if (gesture_id != candidate_) {
        candidate_ = gesture_id;
        candidate_since_s_ = now_s;
    }

    const double held = now_s - candidate_since_s_;
    result.dwell_elapsed_s = held;

    // Stopping is never delayed. Every other change waits out the dwell.
    //
    // The comparison carries a microsecond of slack. The clock is a double of
    // seconds since the epoch, so it is around 1.7e9 and its resolution is a
    // fraction of a microsecond; a decision that lands exactly on the dwell
    // boundary can come out a few hundred nanoseconds short of it through
    // nothing but rounding and holding a gesture for one extra decision
    // because of that would be a real 50 ms of latency for no reason.
    constexpr double kClockSlack = 1e-6;
    const bool immediate = is_rest && config_.rest_is_immediate;
    if (!immediate && held + kClockSlack < config_.dwell_s) {
        result.outcome = MapOutcome::kDwelling;
        return result;
    }

    accepted_ = gesture_id;
    accepted_pose_ = pose->pose;
    result.outcome = MapOutcome::kAccepted;
    result.accepted_gesture = accepted_;
    result.target = accepted_pose_;
    result.moves_arm = pose->moves_arm;
    return result;
}

MapResult GestureMapper::tick(double now_s) {
    MapResult result;
    result.accepted_gesture = accepted_;
    result.target = accepted_pose_;

    if (!started_) {
        result.outcome = MapOutcome::kStale;
        return result;
    }
    if (now_s - last_decision_s_ > config_.staleness_s) {
        // Nothing has arrived for long enough that the classifier or the link
        // is gone. The arm is told to go to rest rather than to hold the last
        // gesture, because holding a grasp with nobody driving it is the
        // failure mode that matters.
        const GesturePose* rest = pose_for(0);
        if (rest != nullptr && accepted_ != 0) {
            accepted_ = 0;
            accepted_pose_ = rest->pose;
            candidate_ = 0;
        }
        result.outcome = MapOutcome::kStale;
        result.accepted_gesture = accepted_;
        result.target = accepted_pose_;
        result.moves_arm = true;
        return result;
    }

    result.outcome = MapOutcome::kNoChange;
    return result;
}

}  // namespace arm_control
