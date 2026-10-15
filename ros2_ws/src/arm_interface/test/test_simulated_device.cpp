#include <cmath>
#include <vector>

#include "arm_interface/device_link.hpp"
#include "test_framework.hpp"

using namespace arm_interface;

namespace {

std::vector<uint8_t> encode_command(const std::array<float, pa::config::kServoChannels>& targets,
                                    uint16_t sequence) {
    pa::protocol::JointCommand command;
    command.target_rad = targets;
    uint8_t body[pa::protocol::kMaxBodyBytes];
    const std::size_t length = pa::protocol::serialise(command, body);
    uint8_t wire[pa::protocol::kMaxEncodedBytes];
    const std::size_t written = pa::protocol::encode_frame(
        static_cast<uint8_t>(pa::protocol::MessageType::kJointCommand), sequence, body, length,
        wire, sizeof(wire));
    return std::vector<uint8_t>(wire, wire + written);
}

// Count the frames of each type in a stream, decoding it the way the host does.
struct FrameCounts {
    int semg{0};
    int telemetry{0};
    int imu{0};
    int errors{0};
    pa::protocol::SemgMessage last_semg{};
    pa::protocol::JointTelemetryMessage last_telemetry{};
    pa::protocol::ImuMessage last_imu{};
};

FrameCounts count(const std::vector<uint8_t>& bytes) {
    FrameCounts counts;
    pa::protocol::FrameDecoder decoder;
    pa::protocol::DecodedFrame frame;
    for (uint8_t byte : bytes) {
        const auto result = decoder.push(byte, frame);
        if (result == pa::protocol::DecodeResult::kIncomplete) continue;
        if (result != pa::protocol::DecodeResult::kFrame) {
            ++counts.errors;
            continue;
        }
        switch (static_cast<pa::protocol::MessageType>(frame.type)) {
            case pa::protocol::MessageType::kSemgFrame:
                if (pa::protocol::deserialise(frame, counts.last_semg)) ++counts.semg;
                break;
            case pa::protocol::MessageType::kJointTelemetry:
                if (pa::protocol::deserialise(frame, counts.last_telemetry)) ++counts.telemetry;
                break;
            case pa::protocol::MessageType::kImuFrame:
                if (pa::protocol::deserialise(frame, counts.last_imu)) ++counts.imu;
                break;
            default:
                ++counts.errors;
                break;
        }
    }
    return counts;
}

// Drive the device for `seconds`, feeding it `targets` at 100 Hz and return
// everything it transmitted.
std::vector<uint8_t> run(SimulatedDevice& device,
                         const std::array<float, pa::config::kServoChannels>& targets,
                         double seconds) {
    std::vector<uint8_t> transmitted;
    const int steps = static_cast<int>(seconds / 1.0e-3);
    uint16_t sequence = 0;
    for (int step = 0; step < steps; ++step) {
        if (step % 10 == 0) {
            const auto command = encode_command(targets, sequence++);
            device.receive(command.data(), command.size());
        }
        device.step();
        const auto out = device.take_transmitted();
        transmitted.insert(transmitted.end(), out.begin(), out.end());
    }
    return transmitted;
}

}  // namespace

TEST(the_device_produces_every_stream_at_the_rate_it_should) {
    const ak::ArmModel model = ak::a1_arm();
    SimulatedDevice device(model, SimulationConfig{});

    std::array<float, pa::config::kServoChannels> targets{};
    const auto bytes = run(device, targets, 1.0);
    const FrameCounts counts = count(bytes);

    CHECK_EQ(counts.errors, 0);
    // 200 Hz, 100 Hz and 100 Hz over one second, give or take the frame that
    // straddles the end.
    CHECK_NEAR(counts.semg, 200, 2);
    CHECK_NEAR(counts.telemetry, 100, 2);
    CHECK_NEAR(counts.imu, 100, 2);
}

TEST(a_command_reaches_the_arm_through_the_real_wire_format) {
    // The point of the whole exercise: the target is encoded by the host's
    // encoder, decoded by the firmware's decoder and the arm ends up there.
    const ak::ArmModel model = ak::a1_arm();
    SimulatedDevice device(model, SimulationConfig{});

    std::array<float, pa::config::kServoChannels> targets{};
    targets[1] = 0.5f;
    targets[2] = -0.9f;
    targets[4] = 0.4f;

    run(device, targets, 4.0);

    const ak::JointVector& q = device.joint_positions();
    CHECK(device.commands_received() > 300);
    CHECK_EQ(static_cast<int>(device.decode_errors()), 0);
    for (std::size_t i = 0; i < ak::kNumJoints; ++i)
        CHECK_NEAR(q[i], static_cast<double>(targets[i]), 0.06);
}

TEST(the_arm_stays_where_it_is_told_against_gravity) {
    // A position servo holds against a load by deflecting until its own
    // stiffness balances it. The residual is the droop the host-side gravity
    // compensation exists to remove and it has to be present here or the
    // compensation would be tested against a model that does not need it.
    const ak::ArmModel model = ak::a1_arm();
    SimulatedDevice device(model, SimulationConfig{});

    std::array<float, pa::config::kServoChannels> targets{};
    targets[1] = 0.0f;  // shoulder horizontal, the worst case
    targets[2] = 1.5707963f;
    run(device, targets, 5.0);

    const ak::JointVector& q = device.joint_positions();
    const double droop = std::fabs(q[1] - targets[1]);
    CHECK(droop > 1e-4);   // there is a load and the servo is not infinitely stiff
    CHECK(droop < 0.20);   // and it is not falling over either
}

TEST(the_outputs_stay_off_until_a_command_arrives) {
    const ak::ArmModel model = ak::a1_arm();
    SimulatedDevice device(model, SimulationConfig{});
    for (int step = 0; step < 100; ++step) device.step();
    CHECK(device.state() == pa::safety::State::kIdle);
    CHECK_EQ(static_cast<int>(device.commands_received()), 0);
}

TEST(the_arm_relaxes_when_the_commands_stop) {
    // The behaviour a dropped link has to produce: hold, then let go. An arm
    // that kept tracking its last target with nobody driving it is the failure
    // this is here to prevent.
    const ak::ArmModel model = ak::a1_arm();
    SimulatedDevice device(model, SimulationConfig{});

    std::array<float, pa::config::kServoChannels> targets{};
    targets[1] = 0.8f;
    targets[2] = -1.0f;
    run(device, targets, 3.0);
    CHECK(device.state() == pa::safety::State::kActive);

    const ak::JointVector held = device.joint_positions();

    // Nothing more arrives.
    for (int step = 0; step < 50; ++step) device.step();
    CHECK(device.state() == pa::safety::State::kHolding);

    for (int step = 0; step < 2000; ++step) device.step();
    CHECK(device.state() == pa::safety::State::kIdle);

    // And having relaxed, it has moved under its own weight.
    double moved = 0.0;
    for (std::size_t i = 0; i < ak::kNumJoints; ++i)
        moved = std::max(moved, std::fabs(device.joint_positions()[i] - held[i]));
    CHECK(moved > 0.05);
}

TEST(the_arm_never_leaves_its_joint_limits) {
    const ak::ArmModel model = ak::a1_arm();
    SimulatedDevice device(model, SimulationConfig{});

    // Ask for far more than the mechanism allows, from every direction.
    for (int round = 0; round < 4; ++round) {
        std::array<float, pa::config::kServoChannels> targets{};
        const float sign = (round % 2 == 0) ? 1.0f : -1.0f;
        for (auto& target : targets) target = sign * 20.0f;
        run(device, targets, 2.0);

        for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
            CHECK(device.joint_positions()[i] >= model.limits[i].lower - 1e-6);
            CHECK(device.joint_positions()[i] <= model.limits[i].upper + 1e-6);
        }
    }
}

TEST(no_joint_is_asked_for_more_torque_than_its_actuator_is_rated_for) {
    // The simulator caps the torque at the rating, so the check that matters
    // is whether the cap is ever reached: if it is, the arm cannot follow the
    // trajectories the planner produces and the actuator choice is wrong.
    const ak::ArmModel model = ak::a1_arm();
    SimulationConfig config;
    SimulatedDevice device(model, config);

    std::array<float, pa::config::kServoChannels> targets{};
    targets[1] = 1.2f;
    targets[2] = -1.4f;
    run(device, targets, 3.0);
    targets[1] = -0.4f;
    targets[2] = 0.9f;
    run(device, targets, 3.0);

    for (std::size_t i = 0; i < ak::kNumJoints; ++i)
        CHECK(device.peak_torque()[i] <= model.limits[i].effort + 1e-9);
}

TEST(the_synthetic_semg_responds_to_the_gesture) {
    const ak::ArmModel model = ak::a1_arm();
    SimulatedDevice device(model, SimulationConfig{});

    std::array<float, pa::config::kServoChannels> targets{};
    const auto resting = count(run(device, targets, 1.0));

    device.set_active_gesture(6);  // fist
    const auto contracting = count(run(device, targets, 1.0));

    double rest_energy = 0.0, active_energy = 0.0;
    for (std::size_t i = 0; i < pa::config::kSemgChannels; ++i) {
        rest_energy += resting.last_semg.envelopes[i];
        active_energy += contracting.last_semg.envelopes[i];
    }
    CHECK(active_energy > rest_energy * 3.0);
}

TEST(different_gestures_produce_different_channel_patterns) {
    // Without this the synthetic signal would be the same on every electrode
    // and any classifier trained on it would be learning amplitude alone.
    const auto fist = activation_for(6);
    const auto point = activation_for(7);

    double difference = 0.0;
    for (std::size_t i = 0; i < pa::config::kSemgChannels; ++i)
        difference += std::fabs(fist.weights[i] - point.weights[i]);
    CHECK(difference > 0.5);

    // And rest recruits nothing at all.
    for (double weight : activation_for(0).weights) CHECK_NEAR(weight, 0.0, 1e-12);
}

TEST(a_lossy_link_costs_frames_and_nothing_else) {
    // Frames go missing, the sequence numbers show the gaps and the arm keeps
    // working. A link that dropped frames and left the arm somewhere unexpected
    // would be the interesting failure; this checks it does not.
    const ak::ArmModel model = ak::a1_arm();
    SimulationConfig config;
    config.frame_loss = 0.15;
    SimulatedDevice device(model, config);

    std::array<float, pa::config::kServoChannels> targets{};
    targets[2] = -0.8f;
    const auto bytes = run(device, targets, 4.0);
    const FrameCounts counts = count(bytes);

    CHECK_EQ(counts.errors, 0);
    CHECK(device.frames_dropped() > 50);
    // What did arrive is intact and the arm went where it was told.
    CHECK_NEAR(device.joint_positions()[2], -0.8, 0.06);
}

TEST(rubbish_on_the_link_is_counted_and_survived) {
    const ak::ArmModel model = ak::a1_arm();
    SimulatedDevice device(model, SimulationConfig{});

    // A burst of noise, then a real command.
    const std::vector<uint8_t> noise(64, 0xA5);
    device.receive(noise.data(), noise.size());
    const uint8_t delimiter = 0x00;
    device.receive(&delimiter, 1);

    std::array<float, pa::config::kServoChannels> targets{};
    targets[4] = 0.6f;
    run(device, targets, 3.0);

    CHECK(device.decode_errors() >= 1);
    CHECK(device.commands_received() > 200);
    CHECK_NEAR(device.joint_positions()[4], 0.6, 0.06);
}

TEST(the_simulated_link_advances_in_whole_steps) {
    const ak::ArmModel model = ak::a1_arm();
    SimulationConfig config;
    SimulatedLink link(model, config);

    // Advanced in awkward slices that do not divide the step. After a second
    // of them the device clock has to agree with the caller's to inside one
    // step, or the timing results the demo reports would be measuring drift.
    for (int i = 0; i < 3000; ++i) link.advance(1.0 / 3000.0);
    CHECK_NEAR(link.device().elapsed_s(), 1.0, config.step_s * 1.5);
}

TEST(a_serial_link_that_cannot_be_opened_says_so) {
    // The only part of the serial path that can be tested without hardware:
    // that a failure to open is reported rather than swallowed and that
    // nothing crashes afterwards.
    SerialLink link("/dev/definitely-not-a-real-port", 921600);
    CHECK(!link.open());
    CHECK(!link.is_open());
    CHECK(!link.last_error().empty());
    CHECK(link.read().empty());
    const uint8_t byte = 0;
    CHECK(!link.write(&byte, 1));
    link.close();
}

TEST_MAIN()
