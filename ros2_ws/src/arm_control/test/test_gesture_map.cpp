#include "arm_control/gesture_map.hpp"
#include "test_framework.hpp"

using namespace arm_control;

namespace {

GestureMapper make_mapper(GestureMapConfig config = {}) {
    return GestureMapper(default_gesture_poses(), config);
}

// Feed the same gesture repeatedly at the classifier's own 50 ms stride.
//
// Returns the acceptance if one happened, rather than whatever the last call
// said. Once a gesture is accepted every further decision for it reports no
// change, so returning the last result would make the answer depend on how
// many decisions the caller happened to send.
MapResult hold(GestureMapper& mapper, uint8_t gesture, double confidence, double& clock,
               int decisions) {
    MapResult result;
    bool accepted = false;
    for (int i = 0; i < decisions; ++i) {
        const MapResult step = mapper.update(gesture, confidence, true, clock);
        clock += 0.05;
        if (!accepted) {
            result = step;
            accepted = step.outcome == MapOutcome::kAccepted;
        }
    }
    return result;
}

}  // namespace

TEST(the_default_table_covers_every_class_the_classifier_can_emit) {
    const auto poses = default_gesture_poses();
    CHECK_EQ(static_cast<int>(poses.size()), static_cast<int>(kGestureCount));
    for (uint8_t id = 0; id < kGestureCount; ++id) {
        bool found = false;
        for (const auto& pose : poses) found = found || pose.id == id;
        CHECK(found);
    }
}

TEST(every_pose_in_the_table_is_inside_the_joint_limits) {
    // A pose the arm cannot reach is a gesture that would be accepted and then
    // silently clamped, which is worse than one that is rejected.
    const ak::ArmModel model = ak::a1_arm();
    for (const auto& pose : default_gesture_poses()) {
        CHECK(ak::within_limits(model, pose.pose, 1e-9));
    }
}

TEST(a_confident_gesture_held_long_enough_is_accepted) {
    GestureMapper mapper = make_mapper();
    double clock = 100.0;

    const MapResult result = hold(mapper, 13, 0.9, clock, 5);
    CHECK(result.outcome == MapOutcome::kAccepted);
    CHECK_EQ(static_cast<int>(result.accepted_gesture), 13);
    CHECK(result.moves_arm);
}

TEST(a_single_misclassification_inside_a_run_changes_nothing) {
    // The reason the dwell exists. At 82 percent accuracy roughly one decision
    // in six is wrong and a wrong decision acted on is a limb moving.
    GestureMapper mapper = make_mapper();
    double clock = 100.0;

    hold(mapper, 13, 0.9, clock, 5);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 13);

    // One stray decision, then back to the real one.
    const MapResult stray = mapper.update(9, 0.95, true, clock);
    clock += 0.05;
    CHECK(stray.outcome == MapOutcome::kDwelling);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 13);

    hold(mapper, 13, 0.9, clock, 3);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 13);
}

TEST(three_consecutive_wrong_decisions_are_still_not_enough) {
    // The dwell is measured from the first decision for the new gesture, so at
    // the classifier's 50 ms stride it takes four of them to span 150 ms, not
    // three. Worth pinning down, because the difference is a whole decision of
    // latency and the obvious reading of "three decisions" is wrong by one.
    GestureMapper mapper = make_mapper();
    double clock = 100.0;
    hold(mapper, 13, 0.9, clock, 5);

    for (int decision = 0; decision < 3; ++decision) {
        const MapResult result = mapper.update(9, 0.95, true, clock);
        clock += 0.05;
        CHECK(result.outcome == MapOutcome::kDwelling);
        CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 13);
    }

    // The fourth, 150 ms after the first, crosses it.
    const MapResult fourth = mapper.update(9, 0.95, true, clock);
    CHECK(fourth.outcome == MapOutcome::kAccepted);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 9);
}

TEST(an_unconfident_decision_is_ignored_and_does_not_count_towards_the_dwell) {
    GestureMapper mapper = make_mapper();
    double clock = 100.0;
    hold(mapper, 13, 0.9, clock, 5);

    // A long run of low-confidence decisions for a different gesture must not
    // accumulate into an acceptance.
    for (int i = 0; i < 20; ++i) {
        const MapResult result = mapper.update(4, 0.3, true, clock);
        clock += 0.05;
        CHECK(result.outcome == MapOutcome::kBelowThreshold);
    }
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 13);
}

TEST(rest_is_never_delayed) {
    // Stopping is the one command that must not wait. Every other change waits
    // out the dwell; this one does not and it does not need to be confident
    // either.
    GestureMapper mapper = make_mapper();
    double clock = 100.0;
    hold(mapper, 6, 0.95, clock, 5);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 6);

    const MapResult result = mapper.update(0, 0.35, true, clock);
    CHECK(result.outcome == MapOutcome::kAccepted);
    CHECK_EQ(static_cast<int>(result.accepted_gesture), 0);
}

TEST(an_invalid_decision_is_not_a_decision) {
    // A lifted electrode produces this. It is different from a low-confidence
    // guess and is treated differently: the candidate is abandoned rather than
    // credited with the time it was held.
    GestureMapper mapper = make_mapper();
    double clock = 100.0;
    hold(mapper, 13, 0.9, clock, 5);

    mapper.update(9, 0.99, true, clock);
    clock += 0.05;
    const MapResult invalid = mapper.update(9, 0.99, false, clock);
    clock += 0.05;
    CHECK(invalid.outcome == MapOutcome::kInvalid);

    // The dwell has been reset, so the next one starts from zero.
    const MapResult next = mapper.update(9, 0.99, true, clock);
    CHECK(next.outcome == MapOutcome::kDwelling);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 13);
}

TEST(an_unknown_class_is_rejected_rather_than_guessed_at) {
    GestureMapper mapper = make_mapper();
    const MapResult result = mapper.update(200, 0.99, true, 100.0);
    CHECK(result.outcome == MapOutcome::kUnknownGesture);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 0);
}

TEST(a_classifier_that_goes_quiet_sends_the_arm_to_rest) {
    GestureMapper mapper = make_mapper();
    double clock = 100.0;
    hold(mapper, 6, 0.95, clock, 5);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 6);

    // Inside the staleness window, nothing changes.
    const MapResult quiet = mapper.tick(clock + 0.3);
    CHECK(quiet.outcome == MapOutcome::kNoChange);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 6);

    // Past it, the arm is told to stop.
    const MapResult stale = mapper.tick(clock + 1.0);
    CHECK(stale.outcome == MapOutcome::kStale);
    CHECK_EQ(static_cast<int>(mapper.accepted_gesture()), 0);
    CHECK(stale.moves_arm);
}

TEST(nothing_at_all_is_treated_as_stale_from_the_start) {
    GestureMapper mapper = make_mapper();
    CHECK(mapper.tick(0.0).outcome == MapOutcome::kStale);
}

TEST(the_same_gesture_repeated_is_reported_as_no_change) {
    GestureMapper mapper = make_mapper();
    double clock = 100.0;
    hold(mapper, 14, 0.9, clock, 5);
    const MapResult again = mapper.update(14, 0.9, true, clock);
    CHECK(again.outcome == MapOutcome::kNoChange);
}

TEST(the_hand_configurations_are_reported_but_do_not_move_the_arm) {
    // This mechanism has no fingers. The gestures that need them are still
    // passed on, because the terminal device can act on them and they are
    // marked so nothing mistakes the substitution for the movement.
    GestureMapper mapper = make_mapper();
    double clock = 100.0;
    const MapResult fist = hold(mapper, 6, 0.95, clock, 5);
    CHECK(fist.outcome == MapOutcome::kAccepted);
    CHECK(!fist.moves_arm);

    const MapResult flexion = hold(mapper, 13, 0.95, clock, 5);
    CHECK(flexion.outcome == MapOutcome::kAccepted);
    CHECK(flexion.moves_arm);
}

TEST(the_wrist_gestures_actually_move_the_wrist) {
    GestureMapper mapper = make_mapper();
    const GesturePose* rest = mapper.pose_for(0);
    const GesturePose* supination = mapper.pose_for(9);
    const GesturePose* pronation = mapper.pose_for(10);
    CHECK(rest != nullptr && supination != nullptr && pronation != nullptr);

    // Forearm rotation, in opposite directions, from the resting pose.
    CHECK(supination->pose[3] > rest->pose[3] + 0.5);
    CHECK(pronation->pose[3] < rest->pose[3] - 0.5);
    // And nothing proximal moves: a wrist gesture should not swing a shoulder.
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK_NEAR(supination->pose[i], rest->pose[i], 1e-12);
        CHECK_NEAR(pronation->pose[i], rest->pose[i], 1e-12);
    }
}

TEST(zero_dwell_makes_every_confident_decision_immediate) {
    // The configuration a bench test would use. Worth checking it does what it
    // says, because a dwell that could not be turned off would make the
    // classifier's own latency impossible to measure separately.
    GestureMapConfig config;
    config.dwell_s = 0.0;
    GestureMapper mapper = make_mapper(config);

    const MapResult result = mapper.update(13, 0.9, true, 100.0);
    CHECK(result.outcome == MapOutcome::kAccepted);
}

TEST(outcome_names_are_all_present) {
    for (MapOutcome outcome : {MapOutcome::kNoChange, MapOutcome::kAccepted,
                               MapOutcome::kBelowThreshold, MapOutcome::kInvalid,
                               MapOutcome::kStale, MapOutcome::kUnknownGesture,
                               MapOutcome::kDwelling}) {
        const char* name = outcome_name(outcome);
        CHECK(name != nullptr);
        CHECK(std::string(name) != "unknown");
    }
}

TEST_MAIN()
