// The fault state machine and the only thing allowed to enable the outputs.
//
// The rules it enforces, in the order they matter:
//
//   1. Outputs are off until a host has said something recent. Power-on is not
//      consent to move.
//   2. A stale command means hold, then relax. Not "carry on with the last
//      target", which is how a dropped link turns into an arm still moving.
//   3. Over-current and over-temperature latch. They are the two conditions
//      that indicate something mechanical is wrong and clearing them
//      automatically would let the arm cycle into the same stall repeatedly.
//   4. Everything else is reported and self-clears.
//
// None of this has been exercised on hardware. It is exercised in
// firmware/test/test_safety.cpp, which is not the same thing.
#pragma once

#include <cstdint>

#include "pa/config/board.hpp"
#include "pa/protocol/messages.hpp"

namespace pa::safety {

enum class State : uint8_t {
    kInit = 0,     // filters not yet verified, nothing enabled
    kIdle,         // healthy, outputs off, waiting for a command
    kActive,       // tracking commands
    kHolding,      // command overdue, holding the last position
    kFault,        // latched fault, outputs off
};

// Everything the supervisor looks at, gathered once per control tick so the
// decision is made on one consistent snapshot rather than on values sampled at
// different points in the loop.
struct Inputs {
    uint32_t now_ms{0};
    uint32_t last_command_ms{0};
    bool ever_commanded{false};
    float bus_current_a{0.0f};
    float board_temperature_c{25.0f};
    uint8_t electrode_saturated_mask{0};
    uint32_t imu_consecutive_failures{0};
    uint32_t link_errors{0};
    bool filters_stable{true};
    bool loop_overrun{false};
};

struct Decision {
    State state{State::kInit};
    uint16_t faults{0};
    bool outputs_enabled{false};
    bool hold_position{false};
};

class Supervisor {
  public:
    Decision update(const Inputs& inputs);

    // Clears the latched faults. Deliberately explicit: a host has to ask,
    // which means a person has looked at the arm.
    void clear_latched();

    State state() const { return state_; }
    uint16_t faults() const { return faults_; }
    uint16_t latched() const { return latched_; }
    uint32_t fault_entries() const { return fault_entries_; }

  private:
    State state_{State::kInit};
    uint16_t faults_{0};
    uint16_t latched_{0};
    uint32_t fault_entries_{0};
    uint32_t link_errors_at_last_check_{0};
};

// How many link errors inside one control period count as the link being
// broken rather than as noise. One corrupted frame on a metre of cable is
// normal; ten in ten milliseconds is not.
inline constexpr uint32_t kLinkErrorBurst = 10;

// Consecutive failed IMU reads before the orientation is declared lost. At
// 100 Hz this is a fifth of a second.
inline constexpr uint32_t kImuFailureLimit = 20;

}  // namespace pa::safety
