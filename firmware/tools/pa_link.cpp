// The host end of the board's serial link, as a command-line tool.
//
// Two jobs, both done with the firmware's own protocol code so that nothing
// here is a second implementation that could drift from the first:
//
//   pa_link commands --out FILE [--start-ms N] [--end-ms N] [--period-ms N]
//                    [--targets a,b,c,d,e,f]
//       Writes a schedule of JointCommand frames for the Renode host-link
//       replay (firmware/renode/peripherals/HostLinkReplay.cs). One line per
//       frame: the virtual time in microseconds, then the encoded bytes in hex.
//       The targets are reached in a straight line from zero over the first
//       half of the window and held for the second.
//
//   pa_link decode FILE
//       Decodes a capture of everything the board transmitted and prints one
//       JSON object per frame, then a summary line with the decoder's error
//       counters. The Renode test reads this.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "pa/protocol/frame.hpp"
#include "pa/protocol/messages.hpp"

namespace {

using namespace pa::protocol;

int usage() {
    std::fprintf(stderr,
                 "usage: pa_link commands --out FILE [--start-ms N] [--end-ms N] [--period-ms N] "
                 "[--targets a,b,c,d,e,f]\n"
                 "       pa_link decode FILE\n");
    return 2;
}

const char* option(int argc, char** argv, const char* name, const char* fallback) {
    for (int i = 2; i + 1 < argc; ++i)
        if (std::strcmp(argv[i], name) == 0) return argv[i + 1];
    return fallback;
}

int write_commands(int argc, char** argv) {
    const char* out_path = option(argc, argv, "--out", nullptr);
    if (out_path == nullptr) return usage();
    const long start_ms = std::atol(option(argc, argv, "--start-ms", "50"));
    const long end_ms = std::atol(option(argc, argv, "--end-ms", "600"));
    const long period_ms = std::atol(option(argc, argv, "--period-ms", "10"));
    if (period_ms <= 0 || end_ms <= start_ms) return usage();

    std::vector<float> targets(pa::config::kServoChannels, 0.0f);
    std::stringstream list(option(argc, argv, "--targets", "0,0,0,0,0,0"));
    std::string item;
    for (std::size_t i = 0; i < targets.size() && std::getline(list, item, ','); ++i)
        targets[i] = std::strtof(item.c_str(), nullptr);

    std::FILE* out = std::fopen(out_path, "wb");
    if (out == nullptr) {
        std::perror(out_path);
        return 1;
    }
    std::fprintf(out, "# JointCommand frames, %ld ms apart, from %ld ms to %ld ms\n", period_ms,
                 start_ms, end_ms);

    const double ramp_ms = static_cast<double>(end_ms - start_ms) / 2.0;
    uint16_t sequence = 0;
    for (long t = start_ms; t <= end_ms; t += period_ms) {
        const double progress = std::fmin(1.0, static_cast<double>(t - start_ms) / ramp_ms);
        JointCommand command;
        for (std::size_t i = 0; i < targets.size(); ++i)
            command.target_rad[i] = static_cast<float>(targets[i] * progress);

        uint8_t body[kMaxBodyBytes];
        const std::size_t length = serialise(command, body);
        uint8_t wire[kMaxEncodedBytes];
        const std::size_t written =
            encode_frame(static_cast<uint8_t>(MessageType::kJointCommand), sequence++, body, length,
                         wire, sizeof(wire));
        if (written == 0) {
            std::fprintf(stderr, "encode_frame failed\n");
            std::fclose(out);
            return 1;
        }
        std::fprintf(out, "%ld ", t * 1000);
        for (std::size_t i = 0; i < written; ++i) std::fprintf(out, "%02x", wire[i]);
        std::fprintf(out, "\n");
    }
    std::fclose(out);
    return 0;
}

template <typename T, std::size_t N>
void print_array(const char* name, const std::array<T, N>& values, bool last = false) {
    std::printf("\"%s\":[", name);
    for (std::size_t i = 0; i < N; ++i)
        std::printf("%s%.7g", i == 0 ? "" : ",", static_cast<double>(values[i]));
    std::printf("]%s", last ? "" : ",");
}

void print_frame(const DecodedFrame& frame) {
    switch (static_cast<MessageType>(frame.type)) {
        case MessageType::kStatus: {
            StatusMessage m;
            if (!deserialise(frame, m)) break;
            std::printf(
                "{\"type\":\"status\",\"seq\":%u,\"uptime_ms\":%lu,\"state\":%u,\"faults\":%u,"
                "\"frames_sent\":%lu,\"commands_received\":%lu,\"link_errors\":%lu,"
                "\"loop_max_us\":%u,\"loop_mean_us\":%u}\n",
                frame.sequence, static_cast<unsigned long>(m.uptime_ms), m.state, m.faults,
                static_cast<unsigned long>(m.frames_sent),
                static_cast<unsigned long>(m.commands_received),
                static_cast<unsigned long>(m.link_errors), m.loop_max_us, m.loop_mean_us);
            return;
        }
        case MessageType::kJointTelemetry: {
            JointTelemetryMessage m;
            if (!deserialise(frame, m)) break;
            std::printf("{\"type\":\"telemetry\",\"seq\":%u,\"timestamp_us\":%lu,", frame.sequence,
                        static_cast<unsigned long>(m.timestamp_us));
            print_array("angle_rad", m.angle_rad);
            print_array("pulse_us", m.pulse_us);
            std::printf("\"bus_current_a\":%.7g,\"temperature_c\":%.1f,\"faults\":%u}\n",
                        static_cast<double>(m.bus_current_a), m.temperature_decic / 10.0, m.faults);
            return;
        }
        case MessageType::kSemgFrame: {
            SemgMessage m;
            if (!deserialise(frame, m)) break;
            std::printf("{\"type\":\"semg\",\"seq\":%u,\"timestamp_us\":%lu,\"sequence\":%lu,",
                        frame.sequence, static_cast<unsigned long>(m.timestamp_us),
                        static_cast<unsigned long>(m.sequence));
            print_array("samples", m.samples);
            print_array("envelopes", m.envelopes);
            std::printf("\"saturated_mask\":%u}\n", m.saturated_mask);
            return;
        }
        case MessageType::kImuFrame: {
            ImuMessage m;
            if (!deserialise(frame, m)) break;
            std::printf("{\"type\":\"imu\",\"seq\":%u,\"timestamp_us\":%lu,", frame.sequence,
                        static_cast<unsigned long>(m.timestamp_us));
            print_array("accel_mps2", m.accel_mps2);
            print_array("gyro_rad_s", m.gyro_rad_s);
            std::printf("\"roll_rad\":%.7g,\"pitch_rad\":%.7g,\"flags\":%u}\n",
                        static_cast<double>(m.roll_rad), static_cast<double>(m.pitch_rad), m.flags);
            return;
        }
        default:
            break;
    }
    std::printf("{\"type\":\"unexpected\",\"seq\":%u,\"message_type\":%u,\"length\":%u}\n",
                frame.sequence, frame.type, frame.length);
}

int decode(int argc, char** argv) {
    if (argc < 3) return usage();
    std::ifstream in(argv[2], std::ios::binary);
    if (!in) {
        std::perror(argv[2]);
        return 1;
    }
    const std::vector<char> bytes((std::istreambuf_iterator<char>(in)),
                                  std::istreambuf_iterator<char>());

    FrameDecoder decoder;
    DecodedFrame frame;
    unsigned long frames = 0;
    for (char c : bytes) {
        if (decoder.push(static_cast<uint8_t>(c), frame) != DecodeResult::kFrame) continue;
        ++frames;
        print_frame(frame);
    }
    std::printf(
        "{\"type\":\"summary\",\"bytes\":%lu,\"frames\":%lu,\"crc_errors\":%lu,\"malformed\":%lu,"
        "\"overflows\":%lu}\n",
        static_cast<unsigned long>(bytes.size()), frames,
        static_cast<unsigned long>(decoder.crc_errors()),
        static_cast<unsigned long>(decoder.malformed()),
        static_cast<unsigned long>(decoder.overflows()));
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    if (std::strcmp(argv[1], "commands") == 0) return write_commands(argc, argv);
    if (std::strcmp(argv[1], "decode") == 0) return decode(argc, argv);
    return usage();
}
