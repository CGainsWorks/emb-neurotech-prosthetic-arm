#include "pa/protocol/frame.hpp"

#include <cstring>

namespace pa::protocol {
namespace {

// Bit-at-a-time rather than a 512-byte table. At a few hundred bytes per
// millisecond this costs under one percent of a 480 MHz core and the table
// would sit in flash competing with the tensor arena.
constexpr uint16_t kPolynomial = 0x1021;

}  // namespace

uint16_t crc16(const uint8_t* data, std::size_t length) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ kPolynomial)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

std::size_t cobs_encode(const uint8_t* data, std::size_t length, uint8_t* out) {
    std::size_t read = 0;
    std::size_t write = 1;
    std::size_t code_index = 0;
    uint8_t code = 1;

    while (read < length) {
        if (data[read] == 0) {
            out[code_index] = code;
            code_index = write++;
            code = 1;
            ++read;
            continue;
        }
        out[write++] = data[read++];
        ++code;
        if (code == 0xFF) {
            out[code_index] = code;
            code_index = write++;
            code = 1;
        }
    }
    out[code_index] = code;
    return write;
}

std::size_t cobs_decode(const uint8_t* data, std::size_t length, uint8_t* out,
                        std::size_t out_capacity) {
    std::size_t read = 0;
    std::size_t write = 0;

    while (read < length) {
        const uint8_t code = data[read];
        if (code == 0) return 0;  // a zero inside the payload cannot happen
        ++read;
        for (uint8_t i = 1; i < code; ++i) {
            if (read >= length) return 0;
            if (write >= out_capacity) return 0;
            out[write++] = data[read++];
        }
        // A group shorter than 255 stood in for a zero, unless it ended the
        // input, in which case there is no trailing zero to restore.
        if (code != 0xFF && read < length) {
            if (write >= out_capacity) return 0;
            out[write++] = 0;
        }
    }
    return write;
}

void put_u16(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value & 0xFF);
    out[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void put_u32(uint8_t* out, uint32_t value) {
    out[0] = static_cast<uint8_t>(value & 0xFF);
    out[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    out[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    out[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

void put_i16(uint8_t* out, int16_t value) { put_u16(out, static_cast<uint16_t>(value)); }

void put_f32(uint8_t* out, float value) {
    // memcpy rather than a union or a pointer cast: it is the only spelling
    // that is not undefined behaviour and every compiler turns it into the
    // same single store.
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    put_u32(out, bits);
}

uint16_t get_u16(const uint8_t* in) {
    return static_cast<uint16_t>(static_cast<uint16_t>(in[0]) |
                                 (static_cast<uint16_t>(in[1]) << 8));
}

uint32_t get_u32(const uint8_t* in) {
    return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
           (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
}

int16_t get_i16(const uint8_t* in) { return static_cast<int16_t>(get_u16(in)); }

float get_f32(const uint8_t* in) {
    const uint32_t bits = get_u32(in);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::size_t encode_frame(uint8_t type, uint16_t sequence, const uint8_t* body,
                         std::size_t body_length, uint8_t* out, std::size_t out_capacity) {
    if (body_length > kMaxBodyBytes) return 0;

    uint8_t raw[kMaxFrameBytes]{};
    raw[0] = type;
    put_u16(raw + 1, sequence);
    put_u16(raw + 3, static_cast<uint16_t>(body_length));
    if (body_length > 0 && body != nullptr) std::memcpy(raw + kHeaderBytes, body, body_length);

    const std::size_t payload = kHeaderBytes + body_length;
    put_u16(raw + payload, crc16(raw, payload));

    const std::size_t total = payload + kCrcBytes;
    if (out_capacity < cobs_encoded_size(total) + 1) return 0;

    const std::size_t encoded = cobs_encode(raw, total, out);
    out[encoded] = kDelimiter;
    return encoded + 1;
}

DecodeResult FrameDecoder::push(uint8_t byte, DecodedFrame& out) {
    if (byte != kDelimiter) {
        if (fill_ < sizeof(buffer_)) {
            buffer_[fill_++] = byte;
        } else if (!overflowed_) {
            // Keep swallowing until the next delimiter rather than reporting
            // once per byte, so one oversized frame produces one error.
            overflowed_ = true;
            ++overflows_;
        }
        return DecodeResult::kIncomplete;
    }

    const std::size_t length = fill_;
    const bool was_overflow = overflowed_;
    fill_ = 0;
    overflowed_ = false;

    if (was_overflow) return DecodeResult::kOverflow;
    // A delimiter with nothing before it is idle line or the tail of a frame
    // already handled, not an error.
    if (length == 0) return DecodeResult::kIncomplete;

    uint8_t raw[kMaxFrameBytes]{};
    const std::size_t decoded = cobs_decode(buffer_, length, raw, sizeof(raw));
    if (decoded < kHeaderBytes + kCrcBytes) {
        ++malformed_;
        return DecodeResult::kMalformed;
    }

    const std::size_t payload = decoded - kCrcBytes;
    const uint16_t expected = get_u16(raw + payload);
    if (crc16(raw, payload) != expected) {
        ++crc_errors_;
        return DecodeResult::kCrcError;
    }

    const uint16_t body_length = get_u16(raw + 3);
    if (static_cast<std::size_t>(body_length) + kHeaderBytes != payload ||
        body_length > kMaxBodyBytes) {
        ++malformed_;
        return DecodeResult::kMalformed;
    }

    out.type = raw[0];
    out.sequence = get_u16(raw + 1);
    out.length = body_length;
    if (body_length > 0) std::memcpy(out.body, raw + kHeaderBytes, body_length);

    ++frames_decoded_;
    return DecodeResult::kFrame;
}

}  // namespace pa::protocol
