// From a gesture class to an arm pose, with the hysteresis that makes it
// usable.
//
// The classifier publishes a decision every 50 ms. Acting on each one directly
// would give an arm that twitches: at 82 percent accuracy roughly one decision
// in six is wrong and a wrong decision that lasts 50 ms is still a limb
// moving. Three things stand between the classifier and the actuators:
//
//   1. A confidence floor. A decision the model is not sure about is not a
//      decision.
//   2. A dwell time. A new gesture has to be held before it is acted on, so an
//      isolated misclassification inside a run of correct ones changes nothing.
//   3. Rest is privileged. Going back to rest is allowed immediately, because
//      stopping is the one command that should never be delayed.
//
// The numbers come from docs/CONTROL_LOOP_TIMING.md, which adds the dwell to
// the classifier's own latency and checks the total against the budget.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "arm_kinematics/model.hpp"

namespace arm_control {

// Ninapro DB5 exercise B numbering: 0 is rest, 1 to 17 are the movements in
// the order the dataset defines them. docs/INTEGRATION.md has the table.
inline constexpr std::size_t kGestureCount = 18;

struct GesturePose {
    uint8_t id{0};
    std::string name;
    ak::JointVector pose{};
    // Gestures that only change the terminal device do not move the arm. The
    // mapper still reports them, so the hand can act on them but it will not
    // plan a trajectory.
    bool moves_arm{true};
};

struct GestureMapConfig {
    // Below this the decision is ignored. 0.6 rather than 0.5: the classifier
    // is calibrated and at 0.5 the false activation rate on DB5 is 7 percent,
    // which on an arm is once every fifteen seconds.
    double confidence_threshold{0.6};

    // How long a new gesture has to be held, measured from the first decision
    // that reports it. 150 ms is three classifier strides, which at 50 ms
    // apiece means the fourth consecutive decision is the one that acts. That
    // is where the number comes from rather than taste and the off-by-one is
    // worth stating because the obvious reading of it is wrong.
    double dwell_s{0.15};

    // A decision older than this is not acted on at all. The link has gone or
    // the classifier has stopped and either way the arm should stop too.
    double staleness_s{0.5};

    // Rest is allowed through without waiting out the dwell.
    bool rest_is_immediate{true};
};

enum class MapOutcome : uint8_t {
    kNoChange,        // same gesture, or nothing new to say
    kAccepted,        // a new gesture has been held long enough to act on
    kBelowThreshold,  // the classifier is not confident enough
    kInvalid,         // the classifier could not decide at all
    kStale,           // nothing has arrived recently enough to trust
    kUnknownGesture,  // a class id with no pose behind it
    kDwelling,        // a candidate is being held but has not been held long enough
};

struct MapResult {
    MapOutcome outcome{MapOutcome::kNoChange};
    uint8_t accepted_gesture{0};
    ak::JointVector target{};
    bool moves_arm{false};
    // How long the current candidate has been held. Published so the reason a
    // gesture has not taken effect is visible rather than mysterious.
    double dwell_elapsed_s{0.0};
};

class GestureMapper {
  public:
    GestureMapper(std::vector<GesturePose> poses, const GestureMapConfig& config);

    // Feed one classifier decision. `now_s` is a monotonic host clock.
    MapResult update(uint8_t gesture_id, double confidence, bool valid, double now_s);

    // Called when no decision has arrived. Reports staleness once the gap is
    // longer than the configured limit.
    MapResult tick(double now_s);

    uint8_t accepted_gesture() const { return accepted_; }
    const ak::JointVector& accepted_pose() const { return accepted_pose_; }

    const std::vector<GesturePose>& poses() const { return poses_; }
    const GesturePose* pose_for(uint8_t gesture_id) const;

  private:
    std::vector<GesturePose> poses_;
    GestureMapConfig config_;

    uint8_t accepted_{0};
    ak::JointVector accepted_pose_{};
    uint8_t candidate_{0};
    double candidate_since_s_{0.0};
    double last_decision_s_{-1.0};
    bool started_{false};
};

// The table the arm ships with.
//
// Most of the eighteen DB5 classes are hand and wrist configurations and a
// six-axis arm has three joints that can express them: forearm rotation, wrist
// flexion and terminal device roll. The finger configurations are reported but
// do not move the arm, because this arm has no fingers. That is a limitation of
// the mechanism, not of the mapping and docs/INTEGRATION.md says so.
std::vector<GesturePose> default_gesture_poses();

const char* outcome_name(MapOutcome outcome);

}  // namespace arm_control
