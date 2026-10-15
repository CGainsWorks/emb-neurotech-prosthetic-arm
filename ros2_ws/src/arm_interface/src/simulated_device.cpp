#include "arm_interface/simulated_device.hpp"

#include <algorithm>
#include <cmath>

namespace arm_interface {
namespace {

constexpr double kSemgPeriod = 1.0 / static_cast<double>(pa::config::kSemgFrameRateHz);
constexpr double kImuPeriod = 1.0 / static_cast<double>(pa::config::kImuRateHz);
constexpr double kTelemetryPeriod = 1.0 / static_cast<double>(pa::config::kJointControlRateHz);

double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

}  // namespace

GestureActivation activation_for(uint8_t gesture_id) {
    GestureActivation activation;
    activation.gesture_id = gesture_id;
    if (gesture_id == 0) return activation;  // rest recruits nothing

    // A deterministic pattern per gesture: each one leans on a different pair
    // of adjacent electrodes with a smaller contribution from the rest, which
    // is roughly what a differential pair over a recruited muscle looks like
    // next to its neighbours. It is a pattern, not a measurement.
    const std::size_t primary = static_cast<std::size_t>(gesture_id) % pa::config::kSemgChannels;
    const std::size_t secondary = (primary + 3) % pa::config::kSemgChannels;
    for (std::size_t i = 0; i < pa::config::kSemgChannels; ++i) {
        if (i == primary) {
            activation.weights[i] = 1.0;
        } else if (i == secondary) {
            activation.weights[i] = 0.65;
        } else {
            activation.weights[i] = 0.15;
        }
    }
    return activation;
}

SimulatedDevice::SimulatedDevice(const ak::ArmModel& model, const SimulationConfig& config)
    : model_(model), config_(config), rng_(config.seed) {
    // Gains from the inertia each joint actually drives, taken at the home
    // configuration. Constant thereafter, as a real servo's are: a servo does
    // not know what the rest of the arm is doing.
    const auto inertia = ak::mass_matrix(model_, ak::JointVector{});
    const double omega = config_.servo_bandwidth_rad_s;
    const double zeta = config_.servo_damping_ratio;
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        const double effective = std::max(inertia(i, i), 1e-4);
        kp_[i] = omega * omega * effective;
        kd_[i] = 2.0 * zeta * omega * effective;
    }

    for (std::size_t i = 0; i < pa::config::kServoChannels; ++i) {
        pa::servo::ChannelCalibration calibration;
        calibration.lower_rad = static_cast<float>(model_.limits[i].lower);
        calibration.upper_rad = static_cast<float>(model_.limits[i].upper);
        calibration.max_rate_rad_s = static_cast<float>(model_.limits[i].velocity);
        servos_.configure(i, calibration);
    }
}

void SimulatedDevice::receive(const uint8_t* data, std::size_t length) {
    pa::protocol::DecodedFrame frame;
    for (std::size_t i = 0; i < length; ++i) {
        const pa::protocol::DecodeResult result = decoder_.push(data[i], frame);
        if (result == pa::protocol::DecodeResult::kIncomplete) continue;
        if (result != pa::protocol::DecodeResult::kFrame) {
            ++decode_errors_;
            continue;
        }

        switch (static_cast<pa::protocol::MessageType>(frame.type)) {
            case pa::protocol::MessageType::kJointCommand: {
                pa::protocol::JointCommand command;
                if (!pa::protocol::deserialise(frame, command)) {
                    ++decode_errors_;
                    break;
                }
                targets_ = command.target_rad;
                last_command_ms_ = static_cast<uint32_t>(elapsed_s_ * 1000.0);
                ever_commanded_ = true;
                ++commands_received_;
                break;
            }
            case pa::protocol::MessageType::kConfigCommand:
                // Accepted and ignored: the simulated front end has no filters
                // to reconfigure and silently doing nothing is better than
                // pretending the setting took effect.
                break;
            default:
                ++decode_errors_;
                break;
        }
    }
}

void SimulatedDevice::set_active_gesture(uint8_t gesture_id) { active_gesture_ = gesture_id; }

void SimulatedDevice::emit(pa::protocol::MessageType type, const uint8_t* body,
                           std::size_t length) {
    // A dropped frame is dropped after encoding, so the sequence number the
    // host sees has a gap in it, which is exactly what a lost frame on a real
    // link looks like.
    if (config_.frame_loss > 0.0) {
        std::uniform_real_distribution<double> chance(0.0, 1.0);
        if (chance(rng_) < config_.frame_loss) {
            ++frames_dropped_;
            ++sequence_;
            return;
        }
    }

    uint8_t wire[pa::protocol::kMaxEncodedBytes];
    const std::size_t written = pa::protocol::encode_frame(static_cast<uint8_t>(type), sequence_++,
                                                           body, length, wire, sizeof(wire));
    if (written == 0) return;
    outgoing_.insert(outgoing_.end(), wire, wire + written);
    ++frames_sent_;
}

void SimulatedDevice::step_dynamics() {
    const double dt = config_.step_s;

    // The safety layer, running on the same inputs the firmware gives it.
    pa::safety::Inputs inputs;
    inputs.now_ms = static_cast<uint32_t>(elapsed_s_ * 1000.0);
    inputs.last_command_ms = last_command_ms_;
    inputs.ever_commanded = ever_commanded_;
    inputs.bus_current_a = 0.0f;
    inputs.board_temperature_c = 30.0f;
    inputs.filters_stable = true;
    const pa::safety::Decision decision = supervisor_.update(inputs);

    std::array<float, pa::config::kServoChannels> commanded{};
    if (decision.outputs_enabled) {
        const pa::servo::DriveState drive =
            decision.hold_position ? servos_.hold()
                                   : servos_.update(targets_, static_cast<float>(dt));
        for (std::size_t i = 0; i < pa::config::kServoChannels; ++i)
            commanded[i] = drive.channels[i].commanded_rad;
    } else {
        // Outputs off: no pulse train, so the servos are back-driveable and
        // the arm falls under its own weight. That is the behaviour, not a
        // simplification and it is what the host's timeout logic exists for.
        ak::JointVector tau{};
        for (std::size_t i = 0; i < ak::kNumJoints; ++i)
            tau[i] = -config_.joint_damping * qd_[i];
        ak::integrate(model_, q_, qd_, tau, dt);
        for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
            const double lower = model_.limits[i].lower;
            const double upper = model_.limits[i].upper;
            if (q_[i] < lower || q_[i] > upper) {
                q_[i] = clamp(q_[i], lower, upper);
                qd_[i] = 0.0;  // it has hit a stop
            }
        }
        last_torque_ = tau;
        return;
    }

    // Each servo is a position loop with a torque ceiling. Gravity is not fed
    // forward here: a servo does not know about it and the droop under load
    // that this produces is the thing the host-side gravity compensation is
    // there to remove.
    ak::JointVector tau{};
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        const double error = static_cast<double>(commanded[i]) - q_[i];
        const double effort = kp_[i] * error - kd_[i] * qd_[i] - config_.joint_damping * qd_[i];
        const double ceiling = model_.limits[i].effort;
        const double limited = clamp(effort, -ceiling, ceiling);
        if (std::fabs(effort) > ceiling) torque_saturated_ = true;
        tau[i] = limited;
        peak_torque_[i] = std::max(peak_torque_[i], std::fabs(limited));
    }

    ak::integrate(model_, q_, qd_, tau, dt);
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        const double lower = model_.limits[i].lower;
        const double upper = model_.limits[i].upper;
        if (q_[i] < lower || q_[i] > upper) {
            q_[i] = clamp(q_[i], lower, upper);
            qd_[i] = 0.0;
        }
    }
    last_torque_ = tau;
}

void SimulatedDevice::step_semg() {
    // Activation follows the gesture with a time constant, because a muscle
    // does not switch on instantly and neither should the signal that stands
    // in for one.
    const double target = active_gesture_ == 0 ? 0.0 : 1.0;
    const double tau = 0.12;
    activation_ += (target - activation_) * (config_.step_s / tau);

    semg_accumulator_ += config_.step_s;
    if (semg_accumulator_ < kSemgPeriod) return;
    semg_accumulator_ -= kSemgPeriod;

    const GestureActivation pattern = activation_for(active_gesture_);

    pa::protocol::SemgMessage message;
    message.timestamp_us = static_cast<uint32_t>(elapsed_s_ * 1e6);
    message.sequence = semg_sequence_++;
    for (std::size_t i = 0; i < pa::config::kSemgChannels; ++i) {
        const double amplitude =
            config_.semg_rest_counts +
            activation_ * pattern.weights[i] * (config_.semg_active_counts - config_.semg_rest_counts);
        const double sample = amplitude * noise_(rng_);
        message.samples[i] = static_cast<int16_t>(clamp(sample, -32767.0, 32767.0));
        // The envelope a real front end would produce is a smoothed rectified
        // signal; here it is the amplitude directly, which is the same thing
        // without the lag.
        message.envelopes[i] = static_cast<uint16_t>(clamp(amplitude, 0.0, 65535.0));
    }

    uint8_t body[pa::protocol::kMaxBodyBytes];
    emit(pa::protocol::MessageType::kSemgFrame, body, pa::protocol::serialise(message, body));
}

void SimulatedDevice::step_imu() {
    imu_accumulator_ += config_.step_s;
    if (imu_accumulator_ < kImuPeriod) return;
    imu_accumulator_ -= kImuPeriod;

    // The IMU is on the forearm, which is link 4 in the chain. Its orientation
    // comes out of the same forward kinematics everything else uses, so the
    // reported roll and pitch are the arm's actual ones rather than a
    // separately invented signal.
    const auto frames = ak::link_transforms(model_, q_);
    const ak::Mat3& rotation = frames[3].rotation;
    const ak::Vec3 gravity_body = rotation.transposed() * model_.gravity;

    pa::protocol::ImuMessage message;
    message.timestamp_us = static_cast<uint32_t>(elapsed_s_ * 1e6);
    for (std::size_t axis = 0; axis < 3; ++axis)
        message.accel_mps2[axis] = static_cast<float>(-gravity_body[axis]);
    message.gyro_rad_s = {{0.0f, 0.0f, 0.0f}};
    message.roll_rad = static_cast<float>(std::atan2(-gravity_body.y, -gravity_body.z));
    message.pitch_rad = static_cast<float>(
        std::atan2(gravity_body.x, std::sqrt(gravity_body.y * gravity_body.y +
                                             gravity_body.z * gravity_body.z)));
    double speed = 0.0;
    for (double rate : qd_) speed = std::max(speed, std::fabs(rate));
    message.flags = static_cast<uint8_t>(1u | (speed < 0.05 ? 2u : 0u));

    uint8_t body[pa::protocol::kMaxBodyBytes];
    emit(pa::protocol::MessageType::kImuFrame, body, pa::protocol::serialise(message, body));
}

void SimulatedDevice::step_telemetry() {
    telemetry_accumulator_ += config_.step_s;
    if (telemetry_accumulator_ < kTelemetryPeriod) return;
    telemetry_accumulator_ -= kTelemetryPeriod;

    pa::protocol::JointTelemetryMessage message;
    message.timestamp_us = static_cast<uint32_t>(elapsed_s_ * 1e6);

    const pa::servo::DriveState drive = servos_.hold();
    for (std::size_t i = 0; i < pa::config::kServoChannels; ++i) {
        // The board reports where the servo was told to go, not where the arm
        // is: a hobby servo has no output-shaft encoder and inventing one
        // here would hide the tracking error the host has to cope with.
        message.angle_rad[i] = drive.channels[i].commanded_rad;
        message.pulse_us[i] = static_cast<uint16_t>(drive.channels[i].pulse_us);
    }
    message.bus_current_a = drive.estimated_current_a;
    message.temperature_decic = 300;  // a fixed 30 C; nothing here models heating
    message.faults = supervisor_.faults();

    uint8_t body[pa::protocol::kMaxBodyBytes];
    emit(pa::protocol::MessageType::kJointTelemetry, body,
         pa::protocol::serialise(message, body));
}

void SimulatedDevice::step() {
    step_dynamics();
    step_semg();
    step_imu();
    step_telemetry();
    elapsed_s_ += config_.step_s;
}

std::vector<uint8_t> SimulatedDevice::take_transmitted() {
    std::vector<uint8_t> out;
    out.swap(outgoing_);
    return out;
}

}  // namespace arm_interface
