// The messages that cross the link and their serialisation.
//
// Every message is packed and unpacked by an explicit sequence of byte writes.
// Nothing here casts a struct to bytes, so the layout is the same whatever
// either end was compiled by and adding a field is a change that shows up in
// a diff of this file rather than one that silently shifts every field after
// it.
//
// The board is the only thing that produces sensor messages and the host is
// the only thing that produces commands. That asymmetry is not enforced by the
// framing; it is what the fault layer assumes, so a command arriving from
// nowhere is a fault, not a surprise.
#pragma once

#include <array>
#include <cstdint>

#include "pa/config/board.hpp"
#include "pa/protocol/frame.hpp"

namespace pa::protocol {

enum class MessageType : uint8_t {
    kJointCommand = 1,     // host to board
    kConfigCommand = 2,    // host to board
    kSemgFrame = 3,        // board to host
    kJointTelemetry = 4,   // board to host
    kImuFrame = 5,         // board to host
    kStatus = 6,           // board to host
};

// Bit positions in the fault word. One bit per condition rather than an
// enumerated state, because faults co-occur and the host wants to see all of
// them, not the first one that happened to be checked.
namespace fault {
inline constexpr uint16_t kCommandTimeout = 1u << 0;
inline constexpr uint16_t kOverCurrent = 1u << 1;
inline constexpr uint16_t kOverTemperature = 1u << 2;
inline constexpr uint16_t kElectrodeSaturated = 1u << 3;
inline constexpr uint16_t kImuLost = 1u << 4;
inline constexpr uint16_t kLinkErrors = 1u << 5;
inline constexpr uint16_t kFilterUnstable = 1u << 6;
inline constexpr uint16_t kCommandRejected = 1u << 7;
inline constexpr uint16_t kOverrun = 1u << 8;
}  // namespace fault

struct JointCommand {
    std::array<float, config::kServoChannels> target_rad{};
    uint8_t enable_mask{0x3F};  // one bit per joint; a clear bit means relax it
    uint8_t flags{0};
    static constexpr std::size_t kBytes = config::kServoChannels * 4 + 2;
};

struct ConfigCommand {
    uint8_t notch_enabled{1};
    uint8_t reserved{0};
    float anti_alias_hz{config::kAntiAliasHz};
    float envelope_hz{config::kEnvelopeHz};
    static constexpr std::size_t kBytes = 10;
};

struct SemgMessage {
    uint32_t timestamp_us{0};
    uint32_t sequence{0};
    std::array<int16_t, config::kSemgChannels> samples{};
    std::array<uint16_t, config::kSemgChannels> envelopes{};
    uint8_t saturated_mask{0};
    static constexpr std::size_t kBytes = 8 + config::kSemgChannels * 4 + 1;
};

struct JointTelemetryMessage {
    uint32_t timestamp_us{0};
    std::array<float, config::kServoChannels> angle_rad{};
    std::array<uint16_t, config::kServoChannels> pulse_us{};
    float bus_current_a{0.0f};
    int16_t temperature_decic{0};  // tenths of a degree Celsius
    uint16_t faults{0};
    static constexpr std::size_t kBytes = 4 + config::kServoChannels * 6 + 8;
};

struct ImuMessage {
    uint32_t timestamp_us{0};
    std::array<float, 3> accel_mps2{};
    std::array<float, 3> gyro_rad_s{};
    float roll_rad{0.0f};
    float pitch_rad{0.0f};
    uint8_t flags{0};  // bit 0: orientation trusted, bit 1: arm stationary
    static constexpr std::size_t kBytes = 37;
};

struct StatusMessage {
    uint32_t uptime_ms{0};
    uint8_t state{0};
    uint16_t faults{0};
    uint32_t frames_sent{0};
    uint32_t commands_received{0};
    uint32_t link_errors{0};
    uint16_t loop_max_us{0};
    uint16_t loop_mean_us{0};
    static constexpr std::size_t kBytes = 23;
};

// Every serialiser writes exactly kBytes and returns that count, so a caller
// can size a buffer from the type alone.
std::size_t serialise(const JointCommand& message, uint8_t* out);
std::size_t serialise(const ConfigCommand& message, uint8_t* out);
std::size_t serialise(const SemgMessage& message, uint8_t* out);
std::size_t serialise(const JointTelemetryMessage& message, uint8_t* out);
std::size_t serialise(const ImuMessage& message, uint8_t* out);
std::size_t serialise(const StatusMessage& message, uint8_t* out);

// Each returns false when the frame is not the right length for the type,
// which is the only way a well-formed frame can still be wrong.
bool deserialise(const DecodedFrame& frame, JointCommand& message);
bool deserialise(const DecodedFrame& frame, ConfigCommand& message);
bool deserialise(const DecodedFrame& frame, SemgMessage& message);
bool deserialise(const DecodedFrame& frame, JointTelemetryMessage& message);
bool deserialise(const DecodedFrame& frame, ImuMessage& message);
bool deserialise(const DecodedFrame& frame, StatusMessage& message);

static_assert(JointTelemetryMessage::kBytes <= kMaxBodyBytes,
              "the frame body buffer must hold the largest message");
static_assert(SemgMessage::kBytes <= kMaxBodyBytes, "");
static_assert(ImuMessage::kBytes <= kMaxBodyBytes, "");

}  // namespace pa::protocol
