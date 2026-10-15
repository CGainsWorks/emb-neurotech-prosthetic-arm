#include "pa/safety/supervisor.hpp"

namespace pa::safety {
namespace {

using namespace pa::protocol::fault;

// The two conditions that mean something mechanical is wrong. They stay set
// until a host explicitly clears them.
constexpr uint16_t kLatching = kOverCurrent | kOverTemperature;

}  // namespace

void Supervisor::clear_latched() {
    latched_ = 0;
    if (state_ == State::kFault) state_ = State::kInit;
}

Decision Supervisor::update(const Inputs& inputs) {
    uint16_t faults = 0;

    if (!inputs.filters_stable) faults |= kFilterUnstable;
    if (inputs.bus_current_a > config::kBusCurrentLimitAmps) faults |= kOverCurrent;
    if (inputs.board_temperature_c > config::kBoardTemperatureLimitC) faults |= kOverTemperature;
    if (inputs.electrode_saturated_mask != 0) faults |= kElectrodeSaturated;
    if (inputs.imu_consecutive_failures >= kImuFailureLimit) faults |= kImuLost;
    if (inputs.loop_overrun) faults |= kOverrun;

    // Link errors are counted as a rate, not a total: a board that has been
    // running for a day has seen a few and that says nothing about now.
    const uint32_t new_link_errors = inputs.link_errors - link_errors_at_last_check_;
    link_errors_at_last_check_ = inputs.link_errors;
    if (new_link_errors >= kLinkErrorBurst) faults |= kLinkErrors;

    // Command age. Unsigned arithmetic on a wrapping millisecond counter is
    // correct here as long as the subtraction is done in the counter's own
    // width, which is what makes this safe across the 49-day wrap.
    const uint32_t command_age_ms = inputs.now_ms - inputs.last_command_ms;
    const bool command_stale = !inputs.ever_commanded || command_age_ms > config::kCommandTimeoutMs;
    if (inputs.ever_commanded && command_stale) faults |= kCommandTimeout;

    latched_ |= faults & kLatching;
    faults_ = faults | latched_;

    Decision decision;
    decision.faults = faults_;

    if (latched_ != 0) {
        if (state_ != State::kFault) ++fault_entries_;
        state_ = State::kFault;
    } else if (!inputs.filters_stable) {
        // Not latched but the arm must not move on filters that could ring:
        // it sits in init until they are redesigned and verified.
        state_ = State::kInit;
    } else if (!inputs.ever_commanded) {
        state_ = State::kIdle;
    } else if (command_stale) {
        // Hold for a grace period, then relax. Holding forever would leave a
        // powered arm pressing against whatever it is touching with nobody
        // watching.
        constexpr uint32_t kRelaxAfterMs = 10 * config::kCommandTimeoutMs;
        state_ = command_age_ms > kRelaxAfterMs ? State::kIdle : State::kHolding;
    } else {
        state_ = State::kActive;
    }

    decision.state = state_;
    decision.outputs_enabled = state_ == State::kActive || state_ == State::kHolding;
    decision.hold_position = state_ == State::kHolding;
    return decision;
}

}  // namespace pa::safety
