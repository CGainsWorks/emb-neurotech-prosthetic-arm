// The whole stack, end to end, in one process, with the numbers written out.
//
// Acquisition through to actuation: a scripted sequence of classifier
// decisions goes into the gesture mapper, the mapper plans, the joint
// controller follows and compensates, the command is encoded onto the wire
// protocol, the simulated board decodes it and the arm moves through the same
// rigid-body dynamics the planner used. Every link in that chain is the real
// one. What is simulated is the board and the wearer and nothing else.
//
// It runs without a middleware on purpose. The same nodes run over DDS from
// arm_bringup/launch/sim.launch.py and that is what RViz talks to; this is the
// version that produces a number, deterministically, in a few seconds, on a CI
// runner with no display and no network.
//
// Two clocks are reported and they measure different things:
//
//   simulated time  what the arm does, which is a property of the design
//   wall clock      what the control computation costs, which is a property of
//                   this machine and is labelled as such
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "arm_control/gesture_map.hpp"
#include "arm_control/joint_controller.hpp"
#include "arm_interface/device_link.hpp"
#include "arm_kinematics/forward_kinematics.hpp"

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kStep = 1.0e-3;
constexpr double kControlPeriod = 0.01;   // 100 Hz
constexpr double kDecisionPeriod = 0.05;  // the classifier's stride

// The scripted wearer. Each entry is a gesture held for a number of seconds,
// with a confidence and a deliberate error rate, so the run exercises the
// gating rather than only the happy path.
struct ScriptEntry {
    double start_s;
    uint8_t gesture;
    double confidence;
};

const ScriptEntry kScript[] = {
    {0.5, 0, 0.95},    // settle at rest
    {1.5, 13, 0.88},   // wrist flexion
    {3.5, 0, 0.93},    // back to rest
    {4.5, 10, 0.81},   // pronation
    {6.5, 9, 0.86},    // supination
    {8.5, 6, 0.91},    // a fist, which this mechanism cannot form
    {10.0, 14, 0.84},  // wrist extension
    {12.0, 0, 0.96},   // rest
};

// One in every this many decisions is deliberately wrong, at high confidence.
// The dwell is what has to absorb them and a run that never produced one
// would not be testing it.
constexpr int kMisclassifyEvery = 7;

struct MoveRecord {
    uint8_t gesture{0};
    double decided_s{0.0};
    double accepted_s{0.0};
    double first_motion_s{-1.0};
    double settled_s{-1.0};
    double peak_error_rad{0.0};
    // Whether this gesture asked the arm to move at all. Nine of the eighteen
    // classes are finger configurations this mechanism cannot form; they are
    // accepted and reported and deliberately move nothing.
    bool planned{false};
    // Whether the wearer asked for something else before this move finished.
    // Not a failure: it is the arm doing as it is told. It does mean the move
    // cannot be held to having settled.
    bool superseded{false};
};

std::string json_array(const std::vector<double>& values, int precision = 6) {
    std::string out = "[";
    char buffer[64];
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) out += ", ";
        std::snprintf(buffer, sizeof(buffer), "%.*g", precision, values[i]);
        out += buffer;
    }
    return out + "]";
}

std::string number(double value, int precision = 6) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*g", precision, value);
    return buffer;
}

double percentile(std::vector<double> values, double fraction) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double position = fraction * static_cast<double>(values.size() - 1);
    const auto lower = static_cast<std::size_t>(position);
    const std::size_t upper = std::min(lower + 1, values.size() - 1);
    const double weight = position - static_cast<double>(lower);
    return values[lower] * (1.0 - weight) + values[upper] * weight;
}

}  // namespace

int main(int argc, char** argv) {
    double duration = 14.0;
    std::string output = "analysis/sim/control_loop_sim.json";
    double frame_loss = 0.0;
    bool compensate = true;
    std::string trace_path;

    for (int i = 1; i + 1 < argc; i += 2) {
        if (std::strcmp(argv[i], "--duration") == 0) duration = std::atof(argv[i + 1]);
        if (std::strcmp(argv[i], "--output") == 0) output = argv[i + 1];
        if (std::strcmp(argv[i], "--frame-loss") == 0) frame_loss = std::atof(argv[i + 1]);
        if (std::strcmp(argv[i], "--no-compensation") == 0) compensate = argv[i + 1][0] == '0';
        if (std::strcmp(argv[i], "--trace") == 0) trace_path = argv[i + 1];
    }

    // Per-tick CSV of what the controller asked for against what the arm did.
    // Not produced unless asked for: it is the thing to look at when a move
    // does not settle and the summary alone cannot say why.
    std::ofstream trace;
    if (!trace_path.empty()) {
        trace.open(trace_path, std::ios::binary);
        trace << "t";
        for (const char* field : {"ref", "cmd", "q", "qd", "tau"})
            for (std::size_t j = 0; j < ak::kNumJoints; ++j) trace << "," << field << j;
        trace << "\n";
    }

    const ak::ArmModel model = ak::a1_arm();

    arm_interface::SimulationConfig sim;
    sim.step_s = kStep;
    sim.frame_loss = frame_loss;
    arm_interface::SimulatedLink link(model, sim);

    arm_control::GestureMapConfig map_config;
    arm_control::GestureMapper mapper(arm_control::default_gesture_poses(), map_config);

    arm_control::ControllerConfig control_config;
    control_config.rate_hz = 1.0 / kControlPeriod;
    // The two corrections are set from the servo model this run is driving,
    // which is what makes the result a test of the controller rather than of a
    // guess. On hardware both numbers come from bring-up instead; the
    // procedure is in firmware/docs/BRING_UP.md and until it has been done
    // the parameters ship at their design values.
    // The compliance is a single number and the servos are not equally stiff,
    // so it is set from the shoulder: that is where the gravity load is and
    // the distal joints carry so little that their compensation is negligible
    // whatever this is set to. The lead is genuinely common to all six.
    control_config.compliance_rad_per_nm = 1.0 / link.device().servo_stiffness()[1];
    control_config.velocity_lead_s = link.device().velocity_lag_s();
    if (!compensate) {
        control_config.compliance_rad_per_nm = 0.0;
        control_config.velocity_lead_s = 0.0;
    }
    arm_control::JointController controller(model, control_config);

    pa::protocol::FrameDecoder decoder;
    ak::JointVector measured{};
    ak::JointVector measured_rate{};

    std::vector<MoveRecord> moves;
    std::vector<double> control_cost_us;
    std::vector<double> plan_cost_us;
    std::vector<double> tracking_error;
    control_cost_us.reserve(static_cast<std::size_t>(duration / kControlPeriod) + 8);

    std::size_t script_index = 0;
    uint8_t performed = 0;
    int decisions = 0;
    int misclassified = 0;
    int accepted = 0;
    int semg_frames = 0;
    int telemetry_frames = 0;
    int imu_frames = 0;

    double next_decision = 0.0;
    double next_control = 0.0;
    uint16_t command_sequence = 0;

    const auto wall_start = Clock::now();

    for (double t = 0.0; t <= duration; t += kStep) {
        // --- the wearer -----------------------------------------------------
        while (script_index < std::size(kScript) && kScript[script_index].start_s <= t) {
            performed = kScript[script_index].gesture;
            link.device().set_active_gesture(performed);
            ++script_index;
        }

        // --- the classifier -------------------------------------------------
        if (t >= next_decision) {
            next_decision += kDecisionPeriod;
            const ScriptEntry& entry =
                kScript[script_index > 0 ? script_index - 1 : 0];

            uint8_t reported = entry.gesture;
            double confidence = entry.confidence;
            ++decisions;
            if (decisions % kMisclassifyEvery == 0 && entry.gesture != 0) {
                // A confident, wrong answer. This is what the dwell exists for.
                reported = static_cast<uint8_t>((entry.gesture % 17) + 1);
                confidence = 0.9;
                ++misclassified;
            }

            const arm_control::MapResult result = mapper.update(reported, confidence, true, t);
            if (result.outcome == arm_control::MapOutcome::kAccepted) {
                ++accepted;
                MoveRecord record;
                record.gesture = result.accepted_gesture;
                record.decided_s = t - result.dwell_elapsed_s;
                record.accepted_s = t;
                moves.push_back(record);

                // Anything still running belongs to the previous gesture and
                // is about to be replaced by this one.
                if (moves.size() > 1) {
                    MoveRecord& previous = moves[moves.size() - 2];
                    if (previous.settled_s < 0.0) previous.superseded = true;
                }

                if (result.moves_arm) {
                    moves.back().planned = true;
                    const auto plan_start = Clock::now();
                    const ak::JointTrajectory plan =
                        ak::plan_joint_move(model, controller.reference(), result.target);
                    const auto plan_end = Clock::now();
                    plan_cost_us.push_back(
                        std::chrono::duration<double, std::micro>(plan_end - plan_start).count());
                    controller.set_trajectory(plan, t);
                }
            }
        }

        // --- the control loop -----------------------------------------------
        if (t >= next_control) {
            next_control += kControlPeriod;

            const auto compute_start = Clock::now();
            const arm_control::ControlCommand command = controller.update(t, measured, measured_rate);
            const auto compute_end = Clock::now();
            control_cost_us.push_back(
                std::chrono::duration<double, std::micro>(compute_end - compute_start).count());

            pa::protocol::JointCommand wire;
            for (std::size_t i = 0; i < pa::config::kServoChannels; ++i)
                wire.target_rad[i] = static_cast<float>(command.position[i]);

            uint8_t body[pa::protocol::kMaxBodyBytes];
            const std::size_t length = pa::protocol::serialise(wire, body);
            uint8_t frame[pa::protocol::kMaxEncodedBytes];
            const std::size_t written = pa::protocol::encode_frame(
                static_cast<uint8_t>(pa::protocol::MessageType::kJointCommand), command_sequence++,
                body, length, frame, sizeof(frame));
            link.write(frame, written);

            double worst = 0.0;
            for (std::size_t i = 0; i < ak::kNumJoints; ++i)
                worst = std::max(worst, std::fabs(command.reference[i] - measured[i]));
            tracking_error.push_back(worst);

            if (trace.is_open()) {
                trace << number(t, 8);
                for (double v : command.reference) trace << "," << number(v, 8);
                for (double v : command.position) trace << "," << number(v, 8);
                for (double v : link.device().joint_positions()) trace << "," << number(v, 8);
                for (double v : link.device().joint_velocities()) trace << "," << number(v, 8);
                for (double v : link.device().applied_torque()) trace << "," << number(v, 8);
                trace << "\n";
            }

            if (!moves.empty()) {
                MoveRecord& current = moves.back();
                if (current.first_motion_s < 0.0) {
                    double motion = 0.0;
                    for (std::size_t i = 0; i < ak::kNumJoints; ++i)
                        motion = std::max(motion,
                                          std::fabs(measured[i] - controller.reference()[i]));
                    if (motion > 1e-3 && t > current.accepted_s) current.first_motion_s = t;
                }
                current.peak_error_rad = std::max(current.peak_error_rad, worst);
                // A degree, which is about the resolution of the servo's own
                // feedback pot and therefore the smallest error worth claiming.
                constexpr double kSettledRad = 0.017;
                if (current.settled_s < 0.0 && !controller.trajectory_active() &&
                    worst < kSettledRad && t > current.accepted_s + 0.05) {
                    current.settled_s = t;
                }
            }
        }

        // --- the board ------------------------------------------------------
        link.advance(kStep);
        for (uint8_t byte : link.read()) {
            pa::protocol::DecodedFrame frame;
            if (decoder.push(byte, frame) != pa::protocol::DecodeResult::kFrame) continue;
            switch (static_cast<pa::protocol::MessageType>(frame.type)) {
                case pa::protocol::MessageType::kJointTelemetry: {
                    pa::protocol::JointTelemetryMessage message;
                    if (!pa::protocol::deserialise(frame, message)) break;
                    ++telemetry_frames;
                    // The board reports where the servo was told to go. The
                    // arm's true position is available here because this is a
                    // simulation and it is used for the tracking error so the
                    // number means something rather than being zero by
                    // construction.
                    measured = link.device().joint_positions();
                    measured_rate = link.device().joint_velocities();
                    break;
                }
                case pa::protocol::MessageType::kSemgFrame: ++semg_frames; break;
                case pa::protocol::MessageType::kImuFrame: ++imu_frames; break;
                default: break;
            }
        }
    }

    const auto wall_end = Clock::now();
    const double wall_s = std::chrono::duration<double>(wall_end - wall_start).count();

    // --- the report ---------------------------------------------------------
    std::vector<double> reaction_ms;
    std::vector<double> settle_ms;
    for (const MoveRecord& move : moves) {
        reaction_ms.push_back((move.accepted_s - move.decided_s) * 1000.0);
        if (move.settled_s > 0.0) settle_ms.push_back((move.settled_s - move.decided_s) * 1000.0);
    }

    std::string moves_json = "[\n";
    for (std::size_t i = 0; i < moves.size(); ++i) {
        const MoveRecord& move = moves[i];
        if (i != 0) moves_json += ",\n";
        moves_json += "      {\"gesture\": " + std::to_string(move.gesture) +
                      ", \"moved_the_arm\": " + (move.planned ? "true" : "false") +
                      ", \"superseded\": " + (move.superseded ? "true" : "false") +
                      ", \"decision_s\": " + number(move.decided_s) +
                      ", \"accepted_s\": " + number(move.accepted_s) +
                      ", \"dwell_ms\": " + number((move.accepted_s - move.decided_s) * 1000.0) +
                      ", \"settled_ms\": " +
                      (move.settled_s > 0.0 ? number((move.settled_s - move.decided_s) * 1000.0)
                                            : std::string("null")) +
                      ", \"peak_tracking_error_rad\": " + number(move.peak_error_rad) + "}";
    }
    moves_json += "\n    ]";

    std::vector<double> peak_torque;
    std::vector<double> torque_rating;
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        peak_torque.push_back(link.device().peak_torque()[i]);
        torque_rating.push_back(model.limits[i].effort);
    }
    std::vector<double> final_pose;
    for (double angle : link.device().joint_positions()) final_pose.push_back(angle);

    const ak::Transform tool = ak::forward_kinematics(model, link.device().joint_positions());

    std::string report;
    report += "{\n";
    report += "  \"report\": \"end to end simulation\",\n";
    report += "  \"source\": \"ros2_ws/src/arm_bringup/src/sim_demo.cpp\",\n";
    report += "  \"disclaimer\": \"Simulated. No hardware exists. The arm, the board and the "
              "wearer are all models; only the control stack, the wire protocol and the "
              "kinematics are the real implementations.\",\n";
    report += "  \"duration_s\": " + number(duration) + ",\n";
    report += "  \"step_s\": " + number(kStep) + ",\n";
    report += "  \"frame_loss\": " + number(frame_loss) + ",\n";
    report += "  \"gravity_compensation\": " + std::string(compensate ? "true" : "false") + ",\n";
    report += "  \"classifier\": {\n";
    report += "    \"decisions\": " + std::to_string(decisions) + ",\n";
    report += "    \"deliberate_misclassifications\": " + std::to_string(misclassified) + ",\n";
    report += "    \"gestures_accepted\": " + std::to_string(accepted) + "\n";
    report += "  },\n";
    report += "  \"frames\": {\n";
    report += "    \"semg\": " + std::to_string(semg_frames) + ",\n";
    report += "    \"telemetry\": " + std::to_string(telemetry_frames) + ",\n";
    report += "    \"imu\": " + std::to_string(imu_frames) + ",\n";
    report += "    \"dropped_by_the_link\": " + std::to_string(link.device().frames_dropped()) +
              ",\n";
    report += "    \"decode_errors\": " + std::to_string(link.device().decode_errors()) + "\n";
    report += "  },\n";
    report += "  \"simulated_time\": {\n";
    report += "    \"note\": \"What the arm does. A property of the design, not of this "
              "machine.\",\n";
    report += "    \"dwell_ms\": " + json_array(reaction_ms) + ",\n";
    report += "    \"settle_from_decision_ms\": " + json_array(settle_ms) + ",\n";
    report += "    \"median_settle_ms\": " + number(percentile(settle_ms, 0.5)) + ",\n";
    report += "    \"max_settle_ms\": " +
              number(settle_ms.empty() ? 0.0
                                       : *std::max_element(settle_ms.begin(), settle_ms.end())) +
              ",\n";
    report += "    \"median_tracking_error_rad\": " + number(percentile(tracking_error, 0.5)) +
              ",\n";
    report += "    \"p99_tracking_error_rad\": " + number(percentile(tracking_error, 0.99)) + ",\n";
    report += "    \"max_tracking_error_rad\": " +
              number(tracking_error.empty()
                         ? 0.0
                         : *std::max_element(tracking_error.begin(), tracking_error.end())) +
              "\n";
    report += "  },\n";
    report += "  \"host_cost\": {\n";
    report += "    \"note\": \"Measured on the machine that ran this, not on the arm's "
              "controller. See docs/CONTROL_LOOP_TIMING.md.\",\n";
    report += "    \"measured\": true,\n";
    report += "    \"control_iterations\": " + std::to_string(control_cost_us.size()) + ",\n";
    report += "    \"control_median_us\": " + number(percentile(control_cost_us, 0.5)) + ",\n";
    report += "    \"control_p99_us\": " + number(percentile(control_cost_us, 0.99)) + ",\n";
    report += "    \"control_max_us\": " +
              number(control_cost_us.empty()
                         ? 0.0
                         : *std::max_element(control_cost_us.begin(), control_cost_us.end())) +
              ",\n";
    report += "    \"plan_median_us\": " + number(percentile(plan_cost_us, 0.5)) + ",\n";
    report += "    \"plan_max_us\": " +
              number(plan_cost_us.empty()
                         ? 0.0
                         : *std::max_element(plan_cost_us.begin(), plan_cost_us.end())) +
              ",\n";
    report += "    \"wall_clock_s\": " + number(wall_s) + ",\n";
    report += "    \"real_time_factor\": " + number(duration / std::max(wall_s, 1e-9)) + "\n";
    report += "  },\n";
    report += "  \"actuation\": {\n";
    report += "    \"peak_torque_nm\": " + json_array(peak_torque) + ",\n";
    report += "    \"rated_torque_nm\": " + json_array(torque_rating) + ",\n";
    report += "    \"torque_saturated_at_any_point\": " +
              std::string(link.device().torque_saturated() ? "true" : "false") + ",\n";
    report += "    \"saturation_note\": \"A position servo reaches its torque limit while "
              "accelerating and that is normal. What would not be normal is reaching it while "
              "holding, which shows up as a move that never settles.\",\n";
    report += "    \"final_joint_rad\": " + json_array(final_pose) + ",\n";
    report += "    \"final_tool_position_m\": " +
              json_array({tool.translation.x, tool.translation.y, tool.translation.z}) + "\n";
    report += "  }\n";
    report += "}\n";

    std::printf("%s", report.c_str());

    if (!output.empty()) {
        std::ofstream file(output, std::ios::binary);
        if (file) {
            file << report;
            std::fprintf(stderr, "wrote %s\n", output.c_str());
        } else {
            std::fprintf(stderr, "could not write %s\n", output.c_str());
            return 1;
        }
    }

    // The run is only meaningful if the pipeline actually carried something.
    if (accepted == 0 || telemetry_frames == 0 || semg_frames == 0) {
        std::fprintf(stderr, "the pipeline did not carry a complete run\n");
        return 1;
    }
    // Every accepted move that asked the arm to go somewhere has to have got
    // there. A run where the arm never settles is a run that proves nothing,
    // and it would otherwise pass quietly.
    std::size_t expected = 0, settled = 0;
    for (const MoveRecord& move : moves) {
        // A gesture this mechanism cannot perform moves nothing and a move
        // the wearer interrupted was never given the chance. Neither is a
        // failure to track and counting them as one would make the check
        // fail for the arm doing exactly as it was told.
        if (!move.planned || move.superseded) continue;
        ++expected;
        if (move.settled_s > 0.0) ++settled;
    }
    if (expected == 0) {
        std::fprintf(stderr, "no move in this run was allowed to finish\n");
        return 1;
    }
    if (settled != expected) {
        std::fprintf(stderr, "%zu of %zu uninterrupted moves never settled\n",
                     expected - settled, expected);
        return 1;
    }
    std::fprintf(stderr, "%zu of %zu uninterrupted moves settled\n", settled, expected);
    return 0;
}
