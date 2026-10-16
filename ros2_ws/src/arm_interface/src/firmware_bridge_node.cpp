// The bridge between ROS 2 and the board.
//
// It owns the link, encodes commands onto it and decodes what comes back. The
// node is deliberately thin: everything it does that could be wrong lives in
// arm_interface's library or in the firmware's own protocol code, both of
// which are unit tested and what is left here is the part that only means
// anything with a middleware running.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "arm_interface/device_link.hpp"
#include "arm_msgs/msg/gesture_command.hpp"
#include "arm_msgs/msg/imu_state.hpp"
#include "arm_msgs/msg/joint_telemetry.hpp"
#include "arm_msgs/msg/semg_frame.hpp"
#include "arm_kinematics/model.hpp"
#include "pa/protocol/messages.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace arm_interface {
namespace {

using namespace std::chrono_literals;

class FirmwareBridge : public rclcpp::Node {
  public:
    FirmwareBridge() : rclcpp::Node("firmware_bridge"), model_(ak::a1_arm()) {
        backend_ = declare_parameter<std::string>("backend", "sim");
        const auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        const auto baud = static_cast<uint32_t>(declare_parameter<int>("baud", 921600));

        SimulationConfig sim;
        sim.step_s = declare_parameter<double>("sim.step_s", 1.0e-3);
        sim.frame_loss = declare_parameter<double>("sim.frame_loss", 0.0);
        sim.seed = static_cast<uint64_t>(declare_parameter<int>("sim.seed", 20260822));

        const double service_rate = declare_parameter<double>("service_rate_hz", 1000.0);
        // Off when something else is the sEMG source, such as arm_bringup's
        // semg_replay, so the two never interleave on one topic. The frames are
        // still decoded and counted; they are just not republished.
        publish_semg_ = declare_parameter<bool>("publish_semg", true);

        link_ = make_link(backend_, port, baud, model_, sim);
        if (!link_->open()) {
            RCLCPP_ERROR(get_logger(), "could not open %s", link_->description().c_str());
        } else {
            RCLCPP_INFO(get_logger(), "bridging to %s", link_->description().c_str());
        }
        if (backend_ != "serial") {
            RCLCPP_WARN(get_logger(),
                        "running against the simulated device: no hardware is involved and "
                        "the sEMG is synthetic");
        }

        for (const auto& name : model_.joint_names) joint_names_.push_back(name);

        joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        telemetry_ = create_publisher<arm_msgs::msg::JointTelemetry>("arm/telemetry", 10);
        semg_ = create_publisher<arm_msgs::msg::SemgFrame>("semg/frame", 50);
        imu_ = create_publisher<arm_msgs::msg::ImuState>("arm/imu", 10);

        command_ = create_subscription<sensor_msgs::msg::JointState>(
            "arm/joint_command", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr message) { on_command(*message); });

        // Only meaningful against the simulated device: it tells the simulated
        // wearer which gesture to perform, which is what shapes the synthetic
        // sEMG. A real board learns nothing from this.
        gesture_ = create_subscription<arm_msgs::msg::GestureCommand>(
            "gesture/command", 10,
            [this](arm_msgs::msg::GestureCommand::SharedPtr message) {
                auto* simulated = dynamic_cast<SimulatedLink*>(link_.get());
                if (simulated != nullptr) simulated->device().set_active_gesture(message->gesture_id);
            });

        const auto period = std::chrono::duration<double>(1.0 / service_rate);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this, service_rate]() { service(1.0 / service_rate); });
    }

  private:
    void on_command(const sensor_msgs::msg::JointState& message) {
        pa::protocol::JointCommand command;
        for (std::size_t i = 0; i < message.name.size() && i < message.position.size(); ++i) {
            const auto it = std::find(joint_names_.begin(), joint_names_.end(), message.name[i]);
            if (it == joint_names_.end()) continue;
            const std::size_t index = static_cast<std::size_t>(std::distance(joint_names_.begin(), it));
            if (index < pa::config::kServoChannels)
                command.target_rad[index] = static_cast<float>(message.position[i]);
        }

        uint8_t body[pa::protocol::kMaxBodyBytes];
        const std::size_t length = pa::protocol::serialise(command, body);
        uint8_t wire[pa::protocol::kMaxEncodedBytes];
        const std::size_t written = pa::protocol::encode_frame(
            static_cast<uint8_t>(pa::protocol::MessageType::kJointCommand), command_sequence_++,
            body, length, wire, sizeof(wire));
        if (written > 0) link_->write(wire, written);
    }

    void service(double dt) {
        link_->advance(dt);
        const std::vector<uint8_t> incoming = link_->read();

        pa::protocol::DecodedFrame frame;
        for (uint8_t byte : incoming) {
            const auto result = decoder_.push(byte, frame);
            if (result != pa::protocol::DecodeResult::kFrame) continue;
            dispatch(frame);
        }
    }

    void dispatch(const pa::protocol::DecodedFrame& frame) {
        const auto now = this->now();
        switch (static_cast<pa::protocol::MessageType>(frame.type)) {
            case pa::protocol::MessageType::kJointTelemetry: {
                pa::protocol::JointTelemetryMessage message;
                if (!pa::protocol::deserialise(frame, message)) return;

                arm_msgs::msg::JointTelemetry out;
                out.header.stamp = now;
                out.header.frame_id = "base_link";
                out.device_timestamp_us = message.timestamp_us;
                for (std::size_t i = 0; i < pa::config::kServoChannels; ++i) {
                    out.commanded_position[i] = message.angle_rad[i];
                    // No output-shaft encoder on this arm, so the measured
                    // position is the commanded one. Reported as such rather
                    // than dressed up: docs/architecture.md says what it would
                    // take to close that loop properly.
                    out.measured_position[i] = message.angle_rad[i];
                    out.measured_velocity[i] = 0.0;
                    out.pulse_us[i] = message.pulse_us[i];
                }
                out.bus_current_a = message.bus_current_a;
                out.board_temperature_c = static_cast<float>(message.temperature_decic) / 10.0f;
                out.faults = message.faults;
                telemetry_->publish(out);

                sensor_msgs::msg::JointState states;
                states.header = out.header;
                states.name = joint_names_;
                states.position.assign(out.measured_position.begin(), out.measured_position.end());
                joint_states_->publish(states);
                return;
            }
            case pa::protocol::MessageType::kSemgFrame: {
                pa::protocol::SemgMessage message;
                if (!pa::protocol::deserialise(frame, message)) return;

                arm_msgs::msg::SemgFrame out;
                out.header.stamp = now;
                out.header.frame_id = "forearm";
                out.sequence = message.sequence;
                out.device_timestamp_us = message.timestamp_us;
                for (std::size_t i = 0; i < pa::config::kSemgChannels; ++i) {
                    out.samples[i] = message.samples[i];
                    out.envelopes[i] = message.envelopes[i];
                }
                out.saturated_mask = message.saturated_mask;
                if (publish_semg_) semg_->publish(out);
                return;
            }
            case pa::protocol::MessageType::kImuFrame: {
                pa::protocol::ImuMessage message;
                if (!pa::protocol::deserialise(frame, message)) return;

                arm_msgs::msg::ImuState out;
                out.header.stamp = now;
                out.header.frame_id = "forearm";
                out.linear_acceleration.x = message.accel_mps2[0];
                out.linear_acceleration.y = message.accel_mps2[1];
                out.linear_acceleration.z = message.accel_mps2[2];
                out.angular_velocity.x = message.gyro_rad_s[0];
                out.angular_velocity.y = message.gyro_rad_s[1];
                out.angular_velocity.z = message.gyro_rad_s[2];
                out.roll = message.roll_rad;
                out.pitch = message.pitch_rad;
                out.trusted = (message.flags & 1u) != 0;
                out.stationary = (message.flags & 2u) != 0;
                imu_->publish(out);
                return;
            }
            default:
                return;
        }
    }

    ak::ArmModel model_;
    std::string backend_;
    std::unique_ptr<DeviceLink> link_;
    pa::protocol::FrameDecoder decoder_;
    std::vector<std::string> joint_names_;
    uint16_t command_sequence_{0};
    bool publish_semg_{true};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
    rclcpp::Publisher<arm_msgs::msg::JointTelemetry>::SharedPtr telemetry_;
    rclcpp::Publisher<arm_msgs::msg::SemgFrame>::SharedPtr semg_;
    rclcpp::Publisher<arm_msgs::msg::ImuState>::SharedPtr imu_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_;
    rclcpp::Subscription<arm_msgs::msg::GestureCommand>::SharedPtr gesture_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace arm_interface

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<arm_interface::FirmwareBridge>());
    rclcpp::shutdown();
    return 0;
}
