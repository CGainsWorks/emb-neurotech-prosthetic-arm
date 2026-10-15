// The 100 Hz loop: follow the trajectory, compensate for gravity, publish the
// command and say what the loop cost.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "arm_control/joint_controller.hpp"
#include "arm_msgs/msg/arm_status.hpp"
#include "arm_msgs/msg/joint_telemetry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace arm_control {
namespace {

using namespace std::chrono_literals;

const char* state_name(bool link_up, bool active) {
    if (!link_up) return "no telemetry";
    return active ? "following a trajectory" : "holding";
}

class JointControllerNode : public rclcpp::Node {
  public:
    JointControllerNode()
        : rclcpp::Node("joint_controller"),
          model_(ak::a1_arm()),
          controller_(model_, read_config(*this)) {
        rate_hz_ = get_parameter("rate_hz").as_double();
        link_timeout_s_ = declare_parameter<double>("link_timeout_s", 0.25);
        for (const auto& name : model_.joint_names) joint_names_.push_back(name);

        command_ = create_publisher<sensor_msgs::msg::JointState>("arm/joint_command", 10);
        status_ = create_publisher<arm_msgs::msg::ArmStatus>("arm/status", 10);

        trajectory_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "arm/trajectory", 10,
            [this](trajectory_msgs::msg::JointTrajectory::SharedPtr message) {
                on_trajectory(*message);
            });

        telemetry_ = create_subscription<arm_msgs::msg::JointTelemetry>(
            "arm/telemetry", 20,
            [this](arm_msgs::msg::JointTelemetry::SharedPtr message) {
                for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
                    measured_[i] = message->measured_position[i];
                    measured_rate_[i] = message->measured_velocity[i];
                }
                faults_ = message->faults;
                last_telemetry_s_ = now().seconds();
            });

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]() { tick(); });

        status_timer_ = create_wall_timer(200ms, [this]() { publish_status(); });
    }

  private:
    static ControllerConfig read_config(rclcpp::Node& node) {
        ControllerConfig config;
        config.rate_hz = node.declare_parameter<double>("rate_hz", config.rate_hz);
        config.compliance_rad_per_nm = node.declare_parameter<double>(
            "compliance_rad_per_nm", config.compliance_rad_per_nm);
        config.velocity_lead_s =
            node.declare_parameter<double>("velocity_lead_s", config.velocity_lead_s);
        config.max_compensation_rad =
            node.declare_parameter<double>("max_compensation_rad", config.max_compensation_rad);
        return config;
    }

    void on_trajectory(const trajectory_msgs::msg::JointTrajectory& message) {
        if (message.points.size() < 2) {
            RCLCPP_WARN(get_logger(), "ignoring a trajectory with %zu points",
                        message.points.size());
            return;
        }
        // Rebuilt as a start, a goal and a duration rather than followed point
        // by point. The points came from the same profile the library
        // generates, so reconstructing it loses nothing and gains a
        // continuously sampleable reference at whatever rate this loop runs at.
        ak::JointVector start{}, goal{};
        const auto& first = message.points.front();
        const auto& last = message.points.back();
        for (std::size_t i = 0; i < ak::kNumJoints && i < first.positions.size(); ++i)
            start[i] = first.positions[i];
        for (std::size_t i = 0; i < ak::kNumJoints && i < last.positions.size(); ++i)
            goal[i] = last.positions[i];

        const double duration = rclcpp::Duration(last.time_from_start).seconds();
        controller_.set_trajectory(ak::JointTrajectory(start, goal, duration,
                                                       ak::ProfileType::kQuintic),
                                   now().seconds());
    }

    void tick() {
        const double t = now().seconds();
        const bool link_up = last_telemetry_s_ > 0.0 && (t - last_telemetry_s_) < link_timeout_s_;
        if (!link_up) {
            // No telemetry means the board or the bridge has gone. Stop
            // following: continuing to send positions to something that is not
            // answering is how a stale command outlives its sender.
            controller_.hold();
        }

        const ControlCommand command = controller_.update(t, measured_, measured_rate_);

        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        message.name = joint_names_;
        message.position.assign(command.position.begin(), command.position.end());
        message.velocity.assign(command.velocity.begin(), command.velocity.end());
        message.effort.assign(command.gravity_torque.begin(), command.gravity_torque.end());
        command_->publish(message);
    }

    void publish_status() {
        const double t = now().seconds();
        const bool link_up = last_telemetry_s_ > 0.0 && (t - last_telemetry_s_) < link_timeout_s_;

        arm_msgs::msg::ArmStatus message;
        message.header.stamp = now();
        message.state = link_up ? (controller_.trajectory_active()
                                       ? arm_msgs::msg::ArmStatus::STATE_ACTIVE
                                       : arm_msgs::msg::ArmStatus::STATE_HOLDING)
                                : arm_msgs::msg::ArmStatus::STATE_IDLE;
        message.state_name = state_name(link_up, controller_.trajectory_active());
        message.faults = faults_;
        message.link_up = link_up;

        const LoopTiming& timing = controller_.timing();
        message.control_period_mean_ms = timing.mean_period_ms();
        message.control_period_max_ms = timing.max_period_ms();
        message.control_jitter_rms_ms = timing.jitter_rms_ms();
        message.control_iterations = timing.iterations();
        message.control_overruns = timing.overruns();
        status_->publish(message);
    }

    ak::ArmModel model_;
    JointController controller_;
    std::vector<std::string> joint_names_;
    ak::JointVector measured_{};
    ak::JointVector measured_rate_{};
    uint16_t faults_{0};
    double rate_hz_{100.0};
    double link_timeout_s_{0.25};
    double last_telemetry_s_{-1.0};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_;
    rclcpp::Publisher<arm_msgs::msg::ArmStatus>::SharedPtr status_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_;
    rclcpp::Subscription<arm_msgs::msg::JointTelemetry>::SharedPtr telemetry_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace
}  // namespace arm_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<arm_control::JointControllerNode>());
    rclcpp::shutdown();
    return 0;
}
