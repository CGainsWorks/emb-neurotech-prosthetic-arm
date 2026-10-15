// The wire format between the board and the host and the streaming decoder
// that recovers frames from it.
//
// This header and its source file are compiled twice: into the firmware and
// into the ROS 2 bridge in ros2_ws/src/arm_interface. There is one
// implementation of the framing on both ends of the link, so an encoder and a
// decoder cannot drift apart and the round-trip test in
// firmware/test/test_protocol.cpp is a test of the real thing rather than of a
// second copy that happens to agree.
//
// Frame layout, before encoding:
//
//   [type u8][sequence u16][length u16][body ...][crc16 u16]
//
// then consistent overhead byte stuffing, then a zero delimiter.
//
// COBS rather than escape characters: it removes every zero from the payload,
// so a single zero byte is an unambiguous frame boundary and a receiver that
// joins mid-stream resynchronises on the next one. Its overhead is one byte
// per frame plus one per 254, which is bounded, unlike escaping, where a
// pathological payload doubles in size and can overflow a fixed buffer.
#pragma once

#include <cstddef>
#include <cstdint>

namespace pa::protocol {

inline constexpr uint8_t kDelimiter = 0x00;
inline constexpr std::size_t kHeaderBytes = 5;
inline constexpr std::size_t kCrcBytes = 2;

// The largest body any message defined in messages.hpp carries, rounded up.
// Static, so every buffer in the firmware is a fixed-size array.
inline constexpr std::size_t kMaxBodyBytes = 64;
inline constexpr std::size_t kMaxFrameBytes = kHeaderBytes + kMaxBodyBytes + kCrcBytes;
// COBS adds one byte, plus one per 254 bytes of payload, plus the delimiter.
inline constexpr std::size_t kMaxEncodedBytes = kMaxFrameBytes + kMaxFrameBytes / 254 + 2;

// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection.
// Chosen over a checksum because it catches every one and two bit error and
// every burst up to sixteen bits, which is what a noisy cable produces.
uint16_t crc16(const uint8_t* data, std::size_t length);

// Encode `length` bytes into `out`, which must hold at least
// cobs_encoded_size(length) bytes. Returns the number written. No delimiter is
// appended; encode_frame does that.
std::size_t cobs_encode(const uint8_t* data, std::size_t length, uint8_t* out);
std::size_t cobs_decode(const uint8_t* data, std::size_t length, uint8_t* out,
                        std::size_t out_capacity);
constexpr std::size_t cobs_encoded_size(std::size_t length) { return length + length / 254 + 1; }

// Build a complete frame, delimiter included. Returns the number of bytes
// written to `out`, or zero if the body is too long or the buffer too small.
std::size_t encode_frame(uint8_t type, uint16_t sequence, const uint8_t* body,
                         std::size_t body_length, uint8_t* out, std::size_t out_capacity);

struct DecodedFrame {
    uint8_t type{0};
    uint16_t sequence{0};
    uint16_t length{0};
    uint8_t body[kMaxBodyBytes]{};
};

enum class DecodeResult : uint8_t {
    kIncomplete,   // no delimiter seen yet
    kFrame,        // a valid frame is in `out`
    kCrcError,
    kMalformed,    // COBS did not decode, or the length field disagrees
    kOverflow,     // more bytes arrived than a frame can hold before a delimiter
};

// Byte-at-a-time decoder. Written this way because that is how bytes actually
// arrive: one per UART interrupt on the board and in arbitrary chunks from a
// serial port on the host. A decoder that needed whole frames handed to it
// would push the resynchronisation problem onto both callers.
class FrameDecoder {
  public:
    DecodeResult push(uint8_t byte, DecodedFrame& out);
    void reset() { fill_ = 0; overflowed_ = false; }

    // Counters, reported in the status message and used by the fault layer:
    // a link that is delivering frames but failing their checks is a different
    // problem to one that has gone quiet.
    uint32_t frames_decoded() const { return frames_decoded_; }
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t malformed() const { return malformed_; }
    uint32_t overflows() const { return overflows_; }

  private:
    uint8_t buffer_[kMaxEncodedBytes]{};
    std::size_t fill_{0};
    bool overflowed_{false};
    uint32_t frames_decoded_{0};
    uint32_t crc_errors_{0};
    uint32_t malformed_{0};
    uint32_t overflows_{0};
};

// Little-endian scalar packing. Explicit byte writes rather than a memcpy of a
// struct: the board is a Cortex-M7 and the host is whatever the user has and
// the two must agree about layout without either of them having to promise
// anything about padding or endianness.
void put_u16(uint8_t* out, uint16_t value);
void put_u32(uint8_t* out, uint32_t value);
void put_i16(uint8_t* out, int16_t value);
void put_f32(uint8_t* out, float value);
uint16_t get_u16(const uint8_t* in);
uint32_t get_u32(const uint8_t* in);
int16_t get_i16(const uint8_t* in);
float get_f32(const uint8_t* in);

}  // namespace pa::protocol
