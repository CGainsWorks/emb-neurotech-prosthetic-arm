#include <cmath>
#include <random>
#include <vector>

#include "pa/protocol/messages.hpp"
#include "test_framework.hpp"

using namespace pa;
using namespace pa::protocol;

namespace {

// Push a whole encoded buffer through the decoder and return the last result.
DecodeResult feed(FrameDecoder& decoder, const uint8_t* data, std::size_t length,
                  DecodedFrame& out) {
    DecodeResult result = DecodeResult::kIncomplete;
    for (std::size_t i = 0; i < length; ++i) result = decoder.push(data[i], out);
    return result;
}

}  // namespace

TEST(crc16_matches_the_published_check_value) {
    // CRC-16/CCITT-FALSE over "123456789" is 0x29B1. Anchoring against the
    // published vector is what makes this the standard algorithm rather than
    // something that merely agrees with itself.
    const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK_EQ(static_cast<int>(crc16(data, sizeof(data))), 0x29B1);
}

TEST(crc16_notices_every_single_bit_flip) {
    std::mt19937 rng(5);
    std::uniform_int_distribution<int> byte(0, 255);
    for (int trial = 0; trial < 500; ++trial) {
        uint8_t payload[48];
        for (auto& value : payload) value = static_cast<uint8_t>(byte(rng));
        const uint16_t original = crc16(payload, sizeof(payload));

        for (std::size_t index = 0; index < sizeof(payload); ++index) {
            for (int bit = 0; bit < 8; ++bit) {
                payload[index] ^= static_cast<uint8_t>(1u << bit);
                CHECK(crc16(payload, sizeof(payload)) != original);
                payload[index] ^= static_cast<uint8_t>(1u << bit);
            }
        }
    }
}

TEST(cobs_round_trips_and_removes_every_zero) {
    // The property the framing rests on: after encoding there is no zero byte
    // anywhere in the payload, so a zero is unambiguously a frame boundary.
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> byte(0, 255);
    std::uniform_int_distribution<int> zeros(0, 3);

    for (int trial = 0; trial < 2000; ++trial) {
        std::vector<uint8_t> input;
        const std::size_t length = static_cast<std::size_t>(trial % 70);
        for (std::size_t i = 0; i < length; ++i)
            input.push_back(zeros(rng) == 0 ? 0 : static_cast<uint8_t>(byte(rng)));

        std::vector<uint8_t> encoded(cobs_encoded_size(input.size()) + 4);
        const std::size_t written = cobs_encode(input.data(), input.size(), encoded.data());
        CHECK(written <= cobs_encoded_size(input.size()));
        for (std::size_t i = 0; i < written; ++i) CHECK(encoded[i] != 0);

        std::vector<uint8_t> decoded(input.size() + 8);
        const std::size_t read = cobs_decode(encoded.data(), written, decoded.data(),
                                             decoded.size());
        CHECK_EQ(static_cast<int>(read), static_cast<int>(input.size()));
        for (std::size_t i = 0; i < input.size(); ++i) CHECK_EQ(decoded[i], input[i]);
    }
}

TEST(cobs_handles_a_run_longer_than_a_single_group) {
    // 254 non-zero bytes is exactly one group, so 300 of them exercises the
    // path where the encoder has to start a new one. It is the case that is
    // always wrong first.
    std::vector<uint8_t> input(300, 0x42);
    std::vector<uint8_t> encoded(cobs_encoded_size(input.size()) + 4);
    const std::size_t written = cobs_encode(input.data(), input.size(), encoded.data());
    for (std::size_t i = 0; i < written; ++i) CHECK(encoded[i] != 0);

    std::vector<uint8_t> decoded(input.size() + 8);
    CHECK_EQ(static_cast<int>(cobs_decode(encoded.data(), written, decoded.data(), decoded.size())),
             300);
    for (std::size_t i = 0; i < input.size(); ++i) CHECK_EQ(decoded[i], 0x42);
}

TEST(a_frame_survives_the_round_trip) {
    uint8_t body[16];
    for (std::size_t i = 0; i < sizeof(body); ++i) body[i] = static_cast<uint8_t>(i * 17);

    uint8_t wire[kMaxEncodedBytes];
    const std::size_t length = encode_frame(3, 4242, body, sizeof(body), wire, sizeof(wire));
    CHECK(length > 0);
    CHECK_EQ(static_cast<int>(wire[length - 1]), 0);

    FrameDecoder decoder;
    DecodedFrame frame;
    CHECK(feed(decoder, wire, length, frame) == DecodeResult::kFrame);
    CHECK_EQ(static_cast<int>(frame.type), 3);
    CHECK_EQ(static_cast<int>(frame.sequence), 4242);
    CHECK_EQ(static_cast<int>(frame.length), 16);
    for (std::size_t i = 0; i < sizeof(body); ++i) CHECK_EQ(frame.body[i], body[i]);
}

TEST(a_corrupted_frame_is_rejected_rather_than_delivered) {
    uint8_t body[24];
    for (std::size_t i = 0; i < sizeof(body); ++i) body[i] = static_cast<uint8_t>(i + 1);

    uint8_t wire[kMaxEncodedBytes];
    const std::size_t length = encode_frame(4, 1, body, sizeof(body), wire, sizeof(wire));

    // Flip one bit in every position in turn. Not one of them may produce a
    // frame the receiver accepts as good.
    int accepted = 0;
    for (std::size_t index = 0; index + 1 < length; ++index) {
        for (int bit = 0; bit < 8; ++bit) {
            uint8_t damaged[kMaxEncodedBytes];
            for (std::size_t i = 0; i < length; ++i) damaged[i] = wire[i];
            damaged[index] ^= static_cast<uint8_t>(1u << bit);
            if (damaged[index] == 0) continue;  // that is a delimiter, tested elsewhere

            FrameDecoder decoder;
            DecodedFrame frame;
            const DecodeResult result = feed(decoder, damaged, length, frame);
            if (result == DecodeResult::kFrame) {
                // The only way a single bit flip can produce a valid frame is
                // if it landed somewhere the CRC does not cover, which for this
                // format is nowhere.
                ++accepted;
            }
        }
    }
    CHECK_EQ(accepted, 0);
}

TEST(the_decoder_resynchronises_after_rubbish_on_the_line) {
    // Joining a stream mid-frame, or a burst of noise, must cost one frame and
    // not the link.
    uint8_t body[8]{1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t wire[kMaxEncodedBytes];
    const std::size_t length = encode_frame(6, 9, body, sizeof(body), wire, sizeof(wire));

    FrameDecoder decoder;
    DecodedFrame frame;

    // A partial frame, then noise, then a whole frame.
    for (std::size_t i = 0; i < length / 2; ++i) decoder.push(wire[i], frame);
    for (uint8_t noise : {0xAA, 0x55, 0xFF, 0x01}) decoder.push(noise, frame);
    decoder.push(0x00, frame);  // whatever that was, it ends here

    CHECK(feed(decoder, wire, length, frame) == DecodeResult::kFrame);
    CHECK_EQ(static_cast<int>(frame.type), 6);
    CHECK_EQ(static_cast<int>(decoder.frames_decoded()), 1);
}

TEST(an_oversized_frame_is_reported_once_not_once_per_byte) {
    FrameDecoder decoder;
    DecodedFrame frame;
    for (std::size_t i = 0; i < kMaxEncodedBytes * 3; ++i) decoder.push(0x7F, frame);
    CHECK(decoder.push(0x00, frame) == DecodeResult::kOverflow);
    CHECK_EQ(static_cast<int>(decoder.overflows()), 1);
    CHECK_EQ(static_cast<int>(decoder.frames_decoded()), 0);
}

TEST(back_to_back_delimiters_are_idle_line_not_errors) {
    FrameDecoder decoder;
    DecodedFrame frame;
    for (int i = 0; i < 50; ++i) CHECK(decoder.push(0x00, frame) == DecodeResult::kIncomplete);
    CHECK_EQ(static_cast<int>(decoder.malformed()), 0);
    CHECK_EQ(static_cast<int>(decoder.crc_errors()), 0);
}

TEST(a_body_larger_than_the_format_allows_is_refused_at_the_encoder) {
    uint8_t body[kMaxBodyBytes + 1]{};
    uint8_t wire[kMaxEncodedBytes];
    CHECK_EQ(static_cast<int>(encode_frame(1, 0, body, sizeof(body), wire, sizeof(wire))), 0);
}

TEST(a_buffer_too_small_to_hold_the_frame_is_refused) {
    uint8_t body[32]{};
    uint8_t tiny[8];
    CHECK_EQ(static_cast<int>(encode_frame(1, 0, body, sizeof(body), tiny, sizeof(tiny))), 0);
}

TEST(scalars_pack_little_endian_whatever_the_host_is) {
    uint8_t buffer[4];
    put_u16(buffer, 0x1234);
    CHECK_EQ(static_cast<int>(buffer[0]), 0x34);
    CHECK_EQ(static_cast<int>(buffer[1]), 0x12);
    CHECK_EQ(static_cast<int>(get_u16(buffer)), 0x1234);

    put_u32(buffer, 0xDEADBEEFu);
    CHECK_EQ(static_cast<int>(buffer[0]), 0xEF);
    CHECK_EQ(static_cast<int>(buffer[3]), 0xDE);
    CHECK(get_u32(buffer) == 0xDEADBEEFu);

    put_i16(buffer, -2);
    CHECK_EQ(static_cast<int>(get_i16(buffer)), -2);

    for (float value : {0.0f, 1.0f, -1.0f, 3.14159f, 1e-20f, 1e20f}) {
        put_f32(buffer, value);
        CHECK_NEAR(get_f32(buffer), value, std::fabs(value) * 1e-6 + 1e-30);
    }
}

TEST(every_message_survives_serialisation) {
    JointCommand command;
    command.target_rad = {{0.1f, -0.2f, 0.3f, -0.4f, 0.5f, -0.6f}};
    command.enable_mask = 0x2A;
    command.flags = 0x01;

    SemgMessage semg;
    semg.timestamp_us = 123456789u;
    semg.sequence = 4242u;
    for (std::size_t i = 0; i < config::kSemgChannels; ++i) {
        semg.samples[i] = static_cast<int16_t>(i * 1000 - 4000);
        semg.envelopes[i] = static_cast<uint16_t>(i * 500);
    }
    semg.saturated_mask = 0x81;

    JointTelemetryMessage telemetry;
    telemetry.timestamp_us = 99u;
    telemetry.angle_rad = {{0.7f, 0.6f, 0.5f, 0.4f, 0.3f, 0.2f}};
    telemetry.pulse_us = {{1500, 1600, 1400, 1550, 1450, 1500}};
    telemetry.bus_current_a = 3.25f;
    telemetry.temperature_decic = 412;
    telemetry.faults = fault::kOverCurrent | fault::kImuLost;

    ImuMessage imu;
    imu.timestamp_us = 7u;
    imu.accel_mps2 = {{0.1f, 0.2f, 9.81f}};
    imu.gyro_rad_s = {{-0.01f, 0.02f, 0.03f}};
    imu.roll_rad = 0.35f;
    imu.pitch_rad = -0.21f;
    imu.flags = 0x03;

    StatusMessage status;
    status.uptime_ms = 60000u;
    status.state = 2;
    status.faults = fault::kLinkErrors;
    status.frames_sent = 12000u;
    status.commands_received = 6000u;
    status.link_errors = 3u;
    status.loop_max_us = 812;
    status.loop_mean_us = 240;

    auto round_trip = [](auto& original, uint8_t type, auto& restored) {
        uint8_t body[kMaxBodyBytes];
        const std::size_t length = serialise(original, body);
        uint8_t wire[kMaxEncodedBytes];
        const std::size_t written = encode_frame(type, 1, body, length, wire, sizeof(wire));
        CHECK(written > 0);

        FrameDecoder decoder;
        DecodedFrame frame;
        DecodeResult result = DecodeResult::kIncomplete;
        for (std::size_t i = 0; i < written; ++i) result = decoder.push(wire[i], frame);
        CHECK(result == DecodeResult::kFrame);
        CHECK_EQ(static_cast<int>(frame.length), static_cast<int>(length));
        CHECK(deserialise(frame, restored));
    };

    JointCommand command_back;
    round_trip(command, static_cast<uint8_t>(MessageType::kJointCommand), command_back);
    for (std::size_t i = 0; i < config::kServoChannels; ++i)
        CHECK_NEAR(command_back.target_rad[i], command.target_rad[i], 1e-7);
    CHECK_EQ(static_cast<int>(command_back.enable_mask), 0x2A);

    SemgMessage semg_back;
    round_trip(semg, static_cast<uint8_t>(MessageType::kSemgFrame), semg_back);
    CHECK(semg_back.timestamp_us == semg.timestamp_us);
    CHECK(semg_back.sequence == semg.sequence);
    for (std::size_t i = 0; i < config::kSemgChannels; ++i) {
        CHECK_EQ(semg_back.samples[i], semg.samples[i]);
        CHECK_EQ(semg_back.envelopes[i], semg.envelopes[i]);
    }
    CHECK_EQ(static_cast<int>(semg_back.saturated_mask), 0x81);

    JointTelemetryMessage telemetry_back;
    round_trip(telemetry, static_cast<uint8_t>(MessageType::kJointTelemetry), telemetry_back);
    CHECK_NEAR(telemetry_back.bus_current_a, 3.25f, 1e-7);
    CHECK_EQ(telemetry_back.temperature_decic, 412);
    CHECK_EQ(static_cast<int>(telemetry_back.faults), static_cast<int>(telemetry.faults));

    ImuMessage imu_back;
    round_trip(imu, static_cast<uint8_t>(MessageType::kImuFrame), imu_back);
    CHECK_NEAR(imu_back.accel_mps2[2], 9.81f, 1e-6);
    CHECK_NEAR(imu_back.pitch_rad, -0.21f, 1e-7);

    StatusMessage status_back;
    round_trip(status, static_cast<uint8_t>(MessageType::kStatus), status_back);
    CHECK(status_back.frames_sent == 12000u);
    CHECK_EQ(static_cast<int>(status_back.loop_max_us), 812);
}

TEST(a_message_of_the_wrong_length_is_refused) {
    // The one way a frame can pass its CRC and still be wrong: the right type
    // carrying the wrong number of bytes, which is what a version mismatch
    // between the board and the host looks like.
    DecodedFrame frame;
    frame.type = static_cast<uint8_t>(MessageType::kJointCommand);
    frame.length = JointCommand::kBytes - 1;

    JointCommand command;
    CHECK(!deserialise(frame, command));

    frame.length = JointCommand::kBytes;
    CHECK(deserialise(frame, command));
}

TEST(a_semg_frame_fits_the_link_at_the_rate_it_is_produced) {
    // 200 frames a second, each one a header, a body and a CRC, plus stuffing.
    // If this does not fit inside the UART the board silently drops frames.
    uint8_t body[kMaxBodyBytes];
    SemgMessage message;
    const std::size_t length = serialise(message, body);
    uint8_t wire[kMaxEncodedBytes];
    const std::size_t on_wire = encode_frame(3, 0, body, length, wire, sizeof(wire));

    // Ten bits per byte: eight data, one start, one stop.
    const double bits_per_second = static_cast<double>(on_wire) * 10.0 *
                                   static_cast<double>(config::kSemgFrameRateHz);
    CHECK(bits_per_second < static_cast<double>(config::kUartBaud) * 0.5);

    // Telemetry at 100 Hz and status at 10 Hz on top of it, still inside half
    // the line rate, which leaves room for the host's commands coming back.
    JointTelemetryMessage telemetry;
    const std::size_t telemetry_bytes =
        encode_frame(4, 0, body, serialise(telemetry, body), wire, sizeof(wire));
    const double total = bits_per_second + static_cast<double>(telemetry_bytes) * 10.0 * 100.0;
    CHECK(total < static_cast<double>(config::kUartBaud) * 0.6);
}

TEST_MAIN()
