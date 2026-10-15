#include "pa/protocol/messages.hpp"

namespace pa::protocol {

std::size_t serialise(const JointCommand& message, uint8_t* out) {
    std::size_t at = 0;
    for (std::size_t i = 0; i < config::kServoChannels; ++i, at += 4)
        put_f32(out + at, message.target_rad[i]);
    out[at++] = message.enable_mask;
    out[at++] = message.flags;
    return at;
}

bool deserialise(const DecodedFrame& frame, JointCommand& message) {
    if (frame.length != JointCommand::kBytes) return false;
    std::size_t at = 0;
    for (std::size_t i = 0; i < config::kServoChannels; ++i, at += 4)
        message.target_rad[i] = get_f32(frame.body + at);
    message.enable_mask = frame.body[at++];
    message.flags = frame.body[at++];
    return true;
}

std::size_t serialise(const ConfigCommand& message, uint8_t* out) {
    out[0] = message.notch_enabled;
    out[1] = message.reserved;
    put_f32(out + 2, message.anti_alias_hz);
    put_f32(out + 6, message.envelope_hz);
    return ConfigCommand::kBytes;
}

bool deserialise(const DecodedFrame& frame, ConfigCommand& message) {
    if (frame.length != ConfigCommand::kBytes) return false;
    message.notch_enabled = frame.body[0];
    message.reserved = frame.body[1];
    message.anti_alias_hz = get_f32(frame.body + 2);
    message.envelope_hz = get_f32(frame.body + 6);
    return true;
}

std::size_t serialise(const SemgMessage& message, uint8_t* out) {
    std::size_t at = 0;
    put_u32(out + at, message.timestamp_us);
    at += 4;
    put_u32(out + at, message.sequence);
    at += 4;
    for (std::size_t i = 0; i < config::kSemgChannels; ++i, at += 2)
        put_i16(out + at, message.samples[i]);
    for (std::size_t i = 0; i < config::kSemgChannels; ++i, at += 2)
        put_u16(out + at, message.envelopes[i]);
    out[at++] = message.saturated_mask;
    return at;
}

bool deserialise(const DecodedFrame& frame, SemgMessage& message) {
    if (frame.length != SemgMessage::kBytes) return false;
    std::size_t at = 0;
    message.timestamp_us = get_u32(frame.body + at);
    at += 4;
    message.sequence = get_u32(frame.body + at);
    at += 4;
    for (std::size_t i = 0; i < config::kSemgChannels; ++i, at += 2)
        message.samples[i] = get_i16(frame.body + at);
    for (std::size_t i = 0; i < config::kSemgChannels; ++i, at += 2)
        message.envelopes[i] = get_u16(frame.body + at);
    message.saturated_mask = frame.body[at++];
    return true;
}

std::size_t serialise(const JointTelemetryMessage& message, uint8_t* out) {
    std::size_t at = 0;
    put_u32(out + at, message.timestamp_us);
    at += 4;
    for (std::size_t i = 0; i < config::kServoChannels; ++i, at += 4)
        put_f32(out + at, message.angle_rad[i]);
    for (std::size_t i = 0; i < config::kServoChannels; ++i, at += 2)
        put_u16(out + at, message.pulse_us[i]);
    put_f32(out + at, message.bus_current_a);
    at += 4;
    put_i16(out + at, message.temperature_decic);
    at += 2;
    put_u16(out + at, message.faults);
    at += 2;
    return at;
}

bool deserialise(const DecodedFrame& frame, JointTelemetryMessage& message) {
    if (frame.length != JointTelemetryMessage::kBytes) return false;
    std::size_t at = 0;
    message.timestamp_us = get_u32(frame.body + at);
    at += 4;
    for (std::size_t i = 0; i < config::kServoChannels; ++i, at += 4)
        message.angle_rad[i] = get_f32(frame.body + at);
    for (std::size_t i = 0; i < config::kServoChannels; ++i, at += 2)
        message.pulse_us[i] = get_u16(frame.body + at);
    message.bus_current_a = get_f32(frame.body + at);
    at += 4;
    message.temperature_decic = get_i16(frame.body + at);
    at += 2;
    message.faults = get_u16(frame.body + at);
    at += 2;
    return true;
}

std::size_t serialise(const ImuMessage& message, uint8_t* out) {
    std::size_t at = 0;
    put_u32(out + at, message.timestamp_us);
    at += 4;
    for (std::size_t i = 0; i < 3; ++i, at += 4) put_f32(out + at, message.accel_mps2[i]);
    for (std::size_t i = 0; i < 3; ++i, at += 4) put_f32(out + at, message.gyro_rad_s[i]);
    put_f32(out + at, message.roll_rad);
    at += 4;
    put_f32(out + at, message.pitch_rad);
    at += 4;
    out[at++] = message.flags;
    return at;
}

bool deserialise(const DecodedFrame& frame, ImuMessage& message) {
    if (frame.length != ImuMessage::kBytes) return false;
    std::size_t at = 0;
    message.timestamp_us = get_u32(frame.body + at);
    at += 4;
    for (std::size_t i = 0; i < 3; ++i, at += 4) message.accel_mps2[i] = get_f32(frame.body + at);
    for (std::size_t i = 0; i < 3; ++i, at += 4) message.gyro_rad_s[i] = get_f32(frame.body + at);
    message.roll_rad = get_f32(frame.body + at);
    at += 4;
    message.pitch_rad = get_f32(frame.body + at);
    at += 4;
    message.flags = frame.body[at++];
    return true;
}

std::size_t serialise(const StatusMessage& message, uint8_t* out) {
    std::size_t at = 0;
    put_u32(out + at, message.uptime_ms);
    at += 4;
    out[at++] = message.state;
    put_u16(out + at, message.faults);
    at += 2;
    put_u32(out + at, message.frames_sent);
    at += 4;
    put_u32(out + at, message.commands_received);
    at += 4;
    put_u32(out + at, message.link_errors);
    at += 4;
    put_u16(out + at, message.loop_max_us);
    at += 2;
    put_u16(out + at, message.loop_mean_us);
    at += 2;
    return at;
}

bool deserialise(const DecodedFrame& frame, StatusMessage& message) {
    if (frame.length != StatusMessage::kBytes) return false;
    std::size_t at = 0;
    message.uptime_ms = get_u32(frame.body + at);
    at += 4;
    message.state = frame.body[at++];
    message.faults = get_u16(frame.body + at);
    at += 2;
    message.frames_sent = get_u32(frame.body + at);
    at += 4;
    message.commands_received = get_u32(frame.body + at);
    at += 4;
    message.link_errors = get_u32(frame.body + at);
    at += 4;
    message.loop_max_us = get_u16(frame.body + at);
    at += 2;
    message.loop_mean_us = get_u16(frame.body + at);
    at += 2;
    return true;
}

}  // namespace pa::protocol
