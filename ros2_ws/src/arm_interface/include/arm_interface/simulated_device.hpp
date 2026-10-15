// A simulated A1 board, speaking the real wire protocol.
//
// This is what makes "tested in simulation" mean something. It is not a stub
// that returns whatever the caller asked for: it takes the same bytes the
// firmware takes, decodes them with the firmware's own decoder, runs the arm
// forward through the same rigid-body dynamics the controller plans against,
// and encodes telemetry back with the firmware's own encoder. A mistake in the
// framing, the field order, the endianness or the message lengths fails here
// exactly as it would on a cable.
//
// What it does not simulate is the analogue front end, the electrodes or the
// physical arm. The sEMG it produces is synthetic and is labelled as such
// everywhere it appears. Nothing in this file is evidence about hardware.
#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "arm_kinematics/dynamics.hpp"
#include "arm_kinematics/model.hpp"
#include "pa/protocol/messages.hpp"
#include "pa/safety/supervisor.hpp"
#include "pa/servo/driver.hpp"

namespace arm_interface {

struct SimulationConfig {
    // The simulator steps at 1 kHz, which is what the firmware's own
    // integrator comment assumes. Faster buys nothing at these masses; slower
    // and semi-implicit Euler starts to lose energy visibly.
    double step_s{1.0e-3};

    // Each servo is modelled as a position loop with a torque ceiling, tuned
    // per joint to its own load. The tuning is given as a closed-loop natural
    // frequency and damping ratio rather than as gains, for two reasons.
    //
    // It is what a servo actually is. A geared hobby servo has a bandwidth of
    // a few hertz under load and the same gains on the shoulder and on the
    // terminal device would leave one ringing and the other sluggish, because
    // the inertias they drive differ by three orders of magnitude.
    //
    // And it is the only way the simulation is stable. Fixed gains stiff
    // enough for the shoulder put the wrist's fastest pole at 30,000 rad/s,
    // which a 1 ms explicit step cannot integrate: the velocity oscillates,
    // the torque sits on its limit and the joint never settles. Deriving the
    // gains from each joint's own inertia keeps every pole inside the step.
    //
    // 50 rad/s is about 8 Hz, which is a good digital servo under a geared
    // load and slightly optimistic for a cheap one. It is also what the
    // distal joints need: their stiffness is the bandwidth squared times the
    // inertia they drive and at 25 rad/s the forearm rotation joint is soft
    // enough that a few tens of millinewton metres of coupling moves it two
    // degrees. Neither number is measured and no servo has been bought.
    //
    // Because the gains scale together with the inertia, every joint ends up
    // with the same closed-loop poles: the same settling time and the same
    // lag while moving. That is what lets the controller correct for the lag
    // with one number instead of six.
    double servo_bandwidth_rad_s{50.0};
    double servo_damping_ratio{1.1};

    // Viscous loss in the geartrain, per joint. Present so the arm settles
    // rather than ringing. Not measured.
    double joint_damping{0.35};

    // Synthetic sEMG. Amplitude in ADC counts at rest and during a
    // contraction and the seed, so a run is reproducible.
    double semg_rest_counts{60.0};
    double semg_active_counts{2200.0};
    uint64_t seed{20260822u};

    // Set non-zero to drop this fraction of outgoing frames, which is what a
    // marginal cable does and what the host's timeout logic has to survive.
    double frame_loss{0.0};
};

// Which channels a gesture recruits, so the synthetic signal has some
// structure rather than being the same noise on every electrode.
struct GestureActivation {
    uint8_t gesture_id{0};
    std::array<double, pa::config::kSemgChannels> weights{};
};

class SimulatedDevice {
  public:
    SimulatedDevice(const ak::ArmModel& model, const SimulationConfig& config);

    // Feed bytes from the host, exactly as a UART would deliver them.
    void receive(const uint8_t* data, std::size_t length);

    // Advance by one step. Anything the board would have transmitted during
    // that step is appended to the outgoing buffer.
    void step();

    // Take everything the board has produced since the last call.
    std::vector<uint8_t> take_transmitted();

    // The gesture currently being "performed" by the wearer, which drives the
    // synthetic sEMG. Set by whatever is scripting the simulation; it is not
    // something the board would know.
    void set_active_gesture(uint8_t gesture_id);

    // Position loop gains, per joint, derived from the inertia each one drives.
    // Published so a controller can be configured to match the servo it is
    // driving, which in simulation is exact and on hardware is what bring-up
    // measures.
    const ak::JointVector& servo_stiffness() const { return kp_; }

    // Seconds the servo lags by while it is moving, which is its own
    // derivative-to-proportional ratio. The same for every joint, because the
    // gains scale together with the inertia.
    double velocity_lag_s() const {
        return 2.0 * config_.servo_damping_ratio / config_.servo_bandwidth_rad_s;
    }

    // --- inspection, for tests and for the demo's report ------------------
    const ak::JointVector& joint_positions() const { return q_; }
    const ak::JointVector& joint_velocities() const { return qd_; }
    ak::JointVector applied_torque() const { return last_torque_; }
    pa::safety::State state() const { return supervisor_.state(); }
    uint16_t faults() const { return supervisor_.faults(); }
    uint32_t commands_received() const { return commands_received_; }
    uint32_t frames_sent() const { return frames_sent_; }
    uint32_t frames_dropped() const { return frames_dropped_; }
    uint32_t decode_errors() const { return decode_errors_; }
    double elapsed_s() const { return elapsed_s_; }

    // Peak torque any joint has been asked for, against its rating. The number
    // that says whether the actuator choice survives the trajectories the
    // planner actually produces.
    const ak::JointVector& peak_torque() const { return peak_torque_; }
    bool torque_saturated() const { return torque_saturated_; }

  private:
    void emit(pa::protocol::MessageType type, const uint8_t* body, std::size_t length);
    void step_dynamics();
    void step_semg();
    void step_imu();
    void step_telemetry();

    ak::ArmModel model_;
    SimulationConfig config_;

    ak::JointVector q_{};
    ak::JointVector qd_{};
    ak::JointVector last_torque_{};
    ak::JointVector peak_torque_{};
    ak::JointVector kp_{};
    ak::JointVector kd_{};
    bool torque_saturated_{false};

    pa::protocol::FrameDecoder decoder_;
    pa::safety::Supervisor supervisor_;
    std::array<float, pa::config::kServoChannels> targets_{};
    pa::servo::Driver servos_;

    std::vector<uint8_t> outgoing_;
    std::mt19937_64 rng_;
    std::normal_distribution<double> noise_{0.0, 1.0};

    double elapsed_s_{0.0};
    double semg_accumulator_{0.0};
    double imu_accumulator_{0.0};
    double telemetry_accumulator_{0.0};
    double activation_{0.0};

    uint8_t active_gesture_{0};
    uint32_t semg_sequence_{0};
    uint16_t sequence_{0};
    uint32_t commands_received_{0};
    uint32_t frames_sent_{0};
    uint32_t frames_dropped_{0};
    uint32_t decode_errors_{0};
    uint32_t last_command_ms_{0};
    bool ever_commanded_{false};
};

// Which electrodes a gesture recruits. Not anatomy: a fixed, documented
// pattern so that different gestures produce distinguishable synthetic
// signals. docs/INTEGRATION.md says why this is not a substitute for a
// recording.
GestureActivation activation_for(uint8_t gesture_id);

}  // namespace arm_interface
