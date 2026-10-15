#include "pa/safety/supervisor.hpp"
#include "test_framework.hpp"

using namespace pa;
using namespace pa::safety;
using namespace pa::protocol;

namespace {

Inputs healthy(uint32_t now_ms) {
    Inputs inputs;
    inputs.now_ms = now_ms;
    inputs.last_command_ms = now_ms;
    inputs.ever_commanded = true;
    inputs.bus_current_a = 1.0f;
    inputs.board_temperature_c = 30.0f;
    inputs.filters_stable = true;
    return inputs;
}

}  // namespace

TEST(the_outputs_are_off_until_a_host_says_something) {
    // Power-on is not consent to move.
    Supervisor supervisor;
    Inputs inputs;
    inputs.now_ms = 0;
    inputs.ever_commanded = false;

    const Decision decision = supervisor.update(inputs);
    CHECK(decision.state == State::kIdle);
    CHECK(!decision.outputs_enabled);
    CHECK_EQ(static_cast<int>(decision.faults), 0);
}

TEST(a_commanded_healthy_arm_is_active) {
    Supervisor supervisor;
    const Decision decision = supervisor.update(healthy(1000));
    CHECK(decision.state == State::kActive);
    CHECK(decision.outputs_enabled);
    CHECK(!decision.hold_position);
    CHECK_EQ(static_cast<int>(decision.faults), 0);
}

TEST(a_stale_command_holds_and_then_relaxes) {
    // The sequence a dropped link produces: hold where you are, then let go.
    // Carrying on with the last target is how a lost host turns into an arm
    // still moving.
    Supervisor supervisor;
    supervisor.update(healthy(1000));

    Inputs stale = healthy(1000);
    stale.now_ms = 1000 + config::kCommandTimeoutMs + 1;
    const Decision holding = supervisor.update(stale);
    CHECK(holding.state == State::kHolding);
    CHECK(holding.hold_position);
    CHECK(holding.outputs_enabled);
    CHECK((holding.faults & fault::kCommandTimeout) != 0);

    stale.now_ms = 1000 + 10 * config::kCommandTimeoutMs + 1;
    const Decision relaxed = supervisor.update(stale);
    CHECK(relaxed.state == State::kIdle);
    CHECK(!relaxed.outputs_enabled);
}

TEST(the_command_timeout_survives_the_millisecond_counter_wrapping) {
    // The counter wraps every 49 days and the arm is meant to run longer than
    // that. Unsigned subtraction in the counter's own width is what makes this
    // correct and it is worth a test because the bug it prevents appears
    // once, weeks in.
    Supervisor supervisor;
    Inputs inputs;
    inputs.ever_commanded = true;
    inputs.filters_stable = true;
    inputs.last_command_ms = 0xFFFFFFF0u;
    inputs.now_ms = 0x00000005u;  // 21 ms later, across the wrap

    const Decision decision = supervisor.update(inputs);
    CHECK(decision.state == State::kActive);
    CHECK_EQ(static_cast<int>(decision.faults & fault::kCommandTimeout), 0);
}

TEST(over_current_latches_and_only_a_host_can_clear_it) {
    // Both latching faults mean something mechanical is wrong. Clearing them
    // automatically would let the arm cycle into the same stall over and over.
    Supervisor supervisor;
    Inputs tripped = healthy(1000);
    tripped.bus_current_a = config::kBusCurrentLimitAmps + 0.5f;

    const Decision faulted = supervisor.update(tripped);
    CHECK(faulted.state == State::kFault);
    CHECK(!faulted.outputs_enabled);
    CHECK((faulted.faults & fault::kOverCurrent) != 0);

    // The current comes back down and the arm stays in fault.
    const Decision still_faulted = supervisor.update(healthy(1010));
    CHECK(still_faulted.state == State::kFault);
    CHECK((still_faulted.faults & fault::kOverCurrent) != 0);

    supervisor.clear_latched();
    const Decision recovered = supervisor.update(healthy(1020));
    CHECK(recovered.state == State::kActive);
    CHECK_EQ(static_cast<int>(recovered.faults), 0);
    CHECK_EQ(static_cast<int>(supervisor.fault_entries()), 1);
}

TEST(over_temperature_latches_the_same_way) {
    Supervisor supervisor;
    Inputs hot = healthy(1000);
    hot.board_temperature_c = config::kBoardTemperatureLimitC + 1.0f;
    CHECK(supervisor.update(hot).state == State::kFault);
    CHECK(supervisor.update(healthy(1010)).state == State::kFault);
    supervisor.clear_latched();
    CHECK(supervisor.update(healthy(1020)).state == State::kActive);
}

TEST(a_lifted_electrode_is_reported_but_does_not_stop_the_arm) {
    // A saturated channel means the classifier's input is rubbish, which the
    // host has to know about. It is not a reason to drop a limb mid-movement.
    Supervisor supervisor;
    Inputs inputs = healthy(1000);
    inputs.electrode_saturated_mask = 0x04;

    const Decision decision = supervisor.update(inputs);
    CHECK((decision.faults & fault::kElectrodeSaturated) != 0);
    CHECK(decision.state == State::kActive);
    CHECK(decision.outputs_enabled);

    // And it clears on its own once the electrode is back.
    CHECK_EQ(static_cast<int>(supervisor.update(healthy(1010)).faults), 0);
}

TEST(a_lost_imu_is_reported_after_a_run_of_failures_not_after_one) {
    // One NACK on a shared bus is normal. Tripping on it would be a fault
    // every few minutes.
    Supervisor supervisor;
    Inputs inputs = healthy(1000);

    inputs.imu_consecutive_failures = kImuFailureLimit - 1;
    CHECK_EQ(static_cast<int>(supervisor.update(inputs).faults & fault::kImuLost), 0);

    inputs.imu_consecutive_failures = kImuFailureLimit;
    CHECK((supervisor.update(inputs).faults & fault::kImuLost) != 0);
}

TEST(link_errors_are_judged_as_a_rate_not_a_total) {
    // A board that has been up for a day has seen a few CRC errors and that
    // says nothing about whether the link is working now.
    Supervisor supervisor;
    Inputs inputs = healthy(1000);

    // A slow trickle, one per tick, forever: never a fault.
    for (uint32_t i = 1; i <= 500; ++i) {
        inputs.now_ms = 1000 + i;
        inputs.last_command_ms = inputs.now_ms;
        inputs.link_errors = i;
        CHECK_EQ(static_cast<int>(supervisor.update(inputs).faults & fault::kLinkErrors), 0);
    }

    // A burst inside one tick: a fault.
    inputs.now_ms = 2000;
    inputs.last_command_ms = 2000;
    inputs.link_errors += kLinkErrorBurst;
    CHECK((supervisor.update(inputs).faults & fault::kLinkErrors) != 0);
}

TEST(unstable_filters_keep_the_arm_in_init) {
    // Not latched but the arm must not move on filters that could ring. It
    // sits in init until they are redesigned and verified.
    Supervisor supervisor;
    Inputs inputs = healthy(1000);
    inputs.filters_stable = false;

    const Decision decision = supervisor.update(inputs);
    CHECK(decision.state == State::kInit);
    CHECK(!decision.outputs_enabled);
    CHECK((decision.faults & fault::kFilterUnstable) != 0);

    CHECK(supervisor.update(healthy(1010)).state == State::kActive);
}

TEST(a_loop_overrun_is_reported) {
    Supervisor supervisor;
    Inputs inputs = healthy(1000);
    inputs.loop_overrun = true;
    CHECK((supervisor.update(inputs).faults & fault::kOverrun) != 0);
    CHECK(supervisor.update(inputs).state == State::kActive);
}

TEST(several_faults_are_reported_together_not_one_at_a_time) {
    // Faults co-occur and a host debugging a board wants all of them, not
    // whichever happened to be checked first.
    Supervisor supervisor;
    Inputs inputs = healthy(1000);
    inputs.electrode_saturated_mask = 0xFF;
    inputs.imu_consecutive_failures = kImuFailureLimit;
    inputs.loop_overrun = true;
    inputs.board_temperature_c = config::kBoardTemperatureLimitC + 5.0f;

    const Decision decision = supervisor.update(inputs);
    CHECK((decision.faults & fault::kElectrodeSaturated) != 0);
    CHECK((decision.faults & fault::kImuLost) != 0);
    CHECK((decision.faults & fault::kOverrun) != 0);
    CHECK((decision.faults & fault::kOverTemperature) != 0);
    CHECK(decision.state == State::kFault);
}

TEST_MAIN()
