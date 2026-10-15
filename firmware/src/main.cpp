// The application. Five periodic tasks and the state that connects them.
//
// Everything with any judgement in it lives in the portable core under
// firmware/src, is compiled into the host test suite and is tested there.
// This file is the wiring: it starts the peripherals, hands their output to
// the core and hands the core's output back to the peripherals. That split is
// deliberate, because none of it has ever run on hardware and the wiring is
// the part that cannot be tested until it does.
//
// See firmware/docs/BRING_UP.md for the order this is meant to be verified in.
#include <array>

#include "pa/app/scheduler.hpp"
#include "pa/config/board.hpp"
#include "pa/imu/lsm6dsox.hpp"
#include "pa/imu/orientation.hpp"
#include "pa/protocol/messages.hpp"
#include "pa/safety/supervisor.hpp"
#include "pa/semg/frontend.hpp"
#include "pa/servo/driver.hpp"
#include "platform/stm32/adc.hpp"
#include "platform/stm32/i2c.hpp"
#include "platform/stm32/pwm_servo.hpp"
#include "platform/stm32/system.hpp"
#include "platform/stm32/uart.hpp"

namespace {

using namespace pa;

semg::FrontEnd g_front_end;
servo::Driver g_servos;
safety::Supervisor g_supervisor;
platform::I2cBus1 g_i2c;
imu::Lsm6dsox g_imu(g_i2c);
imu::ComplementaryFilter g_orientation;

std::array<float, config::kServoChannels> g_targets{};
uint32_t g_last_command_ms = 0;
bool g_ever_commanded = false;
uint32_t g_commands_received = 0;
uint32_t g_frames_sent = 0;
uint16_t g_sequence = 0;
float g_board_temperature_c = 25.0f;
bool g_clocks_ok = false;

// Every joint on this arm is geared differently and every horn is splined, so
// the calibration is per channel and is the first thing bring-up produces.
// These are the design values, not measured ones. There is nothing to measure.
servo::ChannelCalibration calibration_for(std::size_t joint) {
    servo::ChannelCalibration calibration;
    switch (joint) {
        case 0:  // shoulder rotation
            calibration.gear_ratio = 3.0f;
            calibration.lower_rad = -1.75f;
            calibration.upper_rad = 1.75f;
            calibration.max_rate_rad_s = 2.5f;
            calibration.stall_current_a = 2.5f;
            break;
        case 1:  // shoulder flexion
            calibration.gear_ratio = 3.0f;
            calibration.reversed = true;
            calibration.lower_rad = -2.09f;
            calibration.upper_rad = 2.09f;
            calibration.max_rate_rad_s = 2.5f;
            calibration.stall_current_a = 2.5f;
            break;
        case 2:  // elbow flexion
            calibration.gear_ratio = 2.5f;
            calibration.lower_rad = -2.53f;
            calibration.upper_rad = 2.53f;
            calibration.max_rate_rad_s = 2.5f;
            calibration.stall_current_a = 2.0f;
            break;
        case 3:  // forearm rotation
            calibration.gear_ratio = 2.0f;
            calibration.lower_rad = -3.05f;
            calibration.upper_rad = 3.05f;
            calibration.max_rate_rad_s = 4.0f;
            calibration.stall_current_a = 1.2f;
            break;
        case 4:  // wrist flexion
            calibration.gear_ratio = 1.5f;
            calibration.reversed = true;
            calibration.lower_rad = -1.75f;
            calibration.upper_rad = 1.75f;
            calibration.max_rate_rad_s = 4.0f;
            calibration.stall_current_a = 1.2f;
            break;
        default:  // tool roll
            calibration.gear_ratio = 1.0f;
            calibration.lower_rad = -3.05f;
            calibration.upper_rad = 3.05f;
            calibration.max_rate_rad_s = 4.0f;
            calibration.stall_current_a = 0.8f;
            break;
    }
    return calibration;
}

void send(protocol::MessageType type, const uint8_t* body, std::size_t length) {
    uint8_t wire[protocol::kMaxEncodedBytes];
    const std::size_t written = protocol::encode_frame(static_cast<uint8_t>(type), g_sequence++,
                                                       body, length, wire, sizeof(wire));
    if (written > 0 && platform::uart::send(wire, written)) ++g_frames_sent;
}

void handle_commands() {
    protocol::DecodedFrame frame;
    while (platform::uart::poll(frame)) {
        switch (static_cast<protocol::MessageType>(frame.type)) {
            case protocol::MessageType::kJointCommand: {
                protocol::JointCommand command;
                if (!protocol::deserialise(frame, command)) break;
                for (std::size_t i = 0; i < config::kServoChannels; ++i) g_targets[i] = command.target_rad[i];
                g_last_command_ms = platform::millis();
                g_ever_commanded = true;
                ++g_commands_received;
                break;
            }
            case protocol::MessageType::kConfigCommand: {
                protocol::ConfigCommand command;
                if (!protocol::deserialise(frame, command)) break;
                semg::ChannelConfig config;
                config.notch_enabled = command.notch_enabled != 0;
                config.anti_alias_hz = command.anti_alias_hz;
                config.envelope_hz = command.envelope_hz;
                g_front_end.configure(config);
                break;
            }
            default:
                // Sensor messages are produced here, not consumed. One arriving
                // means something else is talking on this link.
                break;
        }
    }
}

void task_acquire() {
    platform::adc::SampleSet raw;
    uint32_t timestamp = 0;
    if (!platform::adc::read(raw, timestamp)) return;

    semg::Frame frame;
    if (!g_front_end.push(raw, timestamp, frame)) return;

    protocol::SemgMessage message;
    message.timestamp_us = frame.timestamp_us;
    message.sequence = frame.sequence;
    message.saturated_mask = frame.saturated_mask;
    for (std::size_t i = 0; i < config::kSemgChannels; ++i) {
        message.samples[i] = frame.samples[i];
        const float envelope = frame.envelopes[i];
        message.envelopes[i] = envelope > 65535.0f ? 65535u : static_cast<uint16_t>(envelope);
    }

    uint8_t body[protocol::kMaxBodyBytes];
    send(protocol::MessageType::kSemgFrame, body, protocol::serialise(message, body));
}

void task_servo(float dt) {
    const servo::DriveState state = platform::pwm::enabled() ? g_servos.update(g_targets, dt)
                                                             : g_servos.hold();
    std::array<float, config::kServoChannels> pulses{};
    for (std::size_t i = 0; i < config::kServoChannels; ++i) pulses[i] = state.channels[i].pulse_us;
    platform::pwm::set_pulses(pulses);
}

void task_control() {
    const servo::DriveState state = g_servos.hold();

    safety::Inputs inputs;
    inputs.now_ms = platform::millis();
    inputs.last_command_ms = g_last_command_ms;
    inputs.ever_commanded = g_ever_commanded;
    inputs.bus_current_a = state.estimated_current_a;
    inputs.board_temperature_c = g_board_temperature_c;
    inputs.imu_consecutive_failures = g_imu.consecutive_failures();
    inputs.link_errors = platform::uart::link_errors();
    inputs.filters_stable = g_front_end.filters_stable() && g_clocks_ok;
    inputs.loop_overrun = platform::adc::dropped() > 0;

    const safety::Decision decision = g_supervisor.update(inputs);
    if (decision.outputs_enabled) {
        platform::pwm::enable();
    } else {
        platform::pwm::disable();
    }
    if (decision.hold_position) {
        g_targets = g_servos.angles();
    }

    protocol::JointTelemetryMessage telemetry;
    telemetry.timestamp_us = platform::micros();
    telemetry.bus_current_a = state.estimated_current_a;
    telemetry.temperature_decic = static_cast<int16_t>(g_board_temperature_c * 10.0f);
    telemetry.faults = decision.faults;
    for (std::size_t i = 0; i < config::kServoChannels; ++i) {
        telemetry.angle_rad[i] = state.channels[i].commanded_rad;
        telemetry.pulse_us[i] = static_cast<uint16_t>(state.channels[i].pulse_us);
    }

    uint8_t body[protocol::kMaxBodyBytes];
    send(protocol::MessageType::kJointTelemetry, body, protocol::serialise(telemetry, body));
}

void task_imu(float dt) {
    imu::Sample sample;
    imu::Orientation orientation;
    if (g_imu.read(sample, platform::micros()) == imu::ImuStatus::kOk) {
        orientation = g_orientation.update(sample, dt);
        // The IMU's own die temperature stands in for the board temperature.
        // It is in the same enclosure and on the same copper; it is not the
        // MOSFET junction the thermal budget is about and the budget says so.
        g_board_temperature_c = sample.temperature_c;
    } else {
        orientation = g_orientation.coast(dt);
        sample = imu::Sample{};
    }

    protocol::ImuMessage message;
    message.timestamp_us = platform::micros();
    for (int axis = 0; axis < 3; ++axis) {
        message.accel_mps2[static_cast<std::size_t>(axis)] = sample.accel_mps2[axis];
        message.gyro_rad_s[static_cast<std::size_t>(axis)] = sample.gyro_rad_s[axis];
    }
    message.roll_rad = orientation.roll_rad;
    message.pitch_rad = orientation.pitch_rad;
    message.flags = static_cast<uint8_t>((orientation.trusted ? 1u : 0u) |
                                         (g_orientation.stationary() ? 2u : 0u));

    uint8_t body[protocol::kMaxBodyBytes];
    send(protocol::MessageType::kImuFrame, body, protocol::serialise(message, body));
}

void task_status(const app::Scheduler& scheduler) {
    protocol::StatusMessage status;
    status.uptime_ms = platform::millis();
    status.state = static_cast<uint8_t>(g_supervisor.state());
    status.faults = g_supervisor.faults();
    status.frames_sent = g_frames_sent;
    status.commands_received = g_commands_received;
    status.link_errors = platform::uart::link_errors() + platform::uart::dropped();
    status.loop_max_us = static_cast<uint16_t>(
        scheduler.stats(app::Task::kControl).max_us() > 65535
            ? 65535
            : scheduler.stats(app::Task::kControl).max_us());
    status.loop_mean_us = static_cast<uint16_t>(scheduler.stats(app::Task::kControl).mean_us());

    uint8_t body[protocol::kMaxBodyBytes];
    send(protocol::MessageType::kStatus, body, protocol::serialise(status, body));
}

}  // namespace

// Called from Reset_Handler. Named app_main rather than main because ISO C++
// forbids taking the address of main and with -nostartfiles the reset vector
// has to do exactly that.
extern "C" int app_main() {
    g_clocks_ok = platform::configure_clocks();

    platform::uart::begin();
    platform::pwm::begin();

    for (std::size_t i = 0; i < config::kServoChannels; ++i)
        g_servos.configure(i, calibration_for(i));
    // Outputs stay off until the supervisor says otherwise, which needs a
    // command from a host. Power-on is not consent to move.
    platform::pwm::disable();

    if (g_i2c.begin()) g_imu.begin();
    platform::adc::begin();

    app::Scheduler scheduler = app::default_scheduler(platform::micros());

    const float servo_dt = 1.0f / config::kServoFrameRateHz;
    const float imu_dt = 1.0f / config::kImuRateHz;

    while (true) {
        const uint32_t now = platform::micros();
        const app::Due due = scheduler.poll(now);

        handle_commands();

        if (due.run[static_cast<std::size_t>(app::Task::kAcquire)]) {
            const uint32_t started = platform::micros();
            task_acquire();
            scheduler.record(app::Task::kAcquire, platform::micros() - started);
        }
        if (due.run[static_cast<std::size_t>(app::Task::kServo)]) {
            const uint32_t started = platform::micros();
            task_servo(servo_dt);
            scheduler.record(app::Task::kServo, platform::micros() - started);
        }
        if (due.run[static_cast<std::size_t>(app::Task::kControl)]) {
            const uint32_t started = platform::micros();
            task_control();
            scheduler.record(app::Task::kControl, platform::micros() - started);
        }
        if (due.run[static_cast<std::size_t>(app::Task::kImu)]) {
            const uint32_t started = platform::micros();
            task_imu(imu_dt);
            scheduler.record(app::Task::kImu, platform::micros() - started);
        }
        if (due.run[static_cast<std::size_t>(app::Task::kStatus)]) {
            const uint32_t started = platform::micros();
            task_status(scheduler);
            scheduler.record(app::Task::kStatus, platform::micros() - started);
        }
    }
}
