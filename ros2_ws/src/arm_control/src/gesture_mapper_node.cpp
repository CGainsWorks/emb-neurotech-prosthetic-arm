// Gesture classes in, joint trajectories out.
//
// Thin on purpose: the gating and the dwell live in arm_control's library and
// are unit tested there and the planning lives in arm_kinematics. What is here
// is the wiring and the service that lets something ask for a Cartesian pose
// directly.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "arm_control/gesture_map.hpp"
#include "arm_kinematics/inverse_kinematics.hpp"
#include "arm_kinematics/trajectory.hpp"
#include "arm_msgs/msg/gesture_command.hpp"
#include "arm_msgs/srv/move_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace arm_control {
namespace {

using namespace std::chrono_literals;

trajectory_msgs::msg::JointTrajectory to_message(const std::vector<std::string>& names,
                                                 const ak::JointTrajectory& trajectory,
                                                 int samples) {
    trajectory_msgs::msg::JointTrajectory message;
    message.joint_names = names;
    for (const auto& point : trajectory.discretise(samples)) {
        trajectory_msgs::msg::JointTrajectoryPoint out;
        out.positions.assign(point.q.begin(), point.q.end());
        out.velocities.assign(point.qd.begin(), point.qd.end());
        out.accelerations.assign(point.qdd.begin(), point.qdd.end());
        out.time_from_start = rclcpp::Duration::from_seconds(point.time);
        message.points.push_back(out);
    }
    return message;
}

class GestureMapperNode : public rclcpp::Node {
  public:
    GestureMapperNode()
        : rclcpp::Node("gesture_mapper"),
          model_(ak::a1_arm()),
          mapper_(default_gesture_poses(), read_config(*this)) {
        samples_ = declare_parameter<int>("trajectory_samples", 40);
        for (const auto& name : model_.joint_names) joint_names_.push_back(name);

        trajectory_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("arm/trajectory", 10);

        gesture_ = create_subscription<arm_msgs::msg::GestureCommand>(
            "gesture/command", 20,
            [this](arm_msgs::msg::GestureCommand::SharedPtr message) { on_gesture(*message); });

        joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr message) {
                for (std::size_t i = 0; i < message->name.size() && i < message->position.size();
                     ++i) {
                    const auto it =
                        std::find(joint_names_.begin(), joint_names_.end(), message->name[i]);
                    if (it == joint_names_.end()) continue;
                    const auto index = static_cast<std::size_t>(
                        std::distance(joint_names_.begin(), it));
                    if (index < ak::kNumJoints) current_[index] = message->position[i];
                }
                have_state_ = true;
            });

        move_service_ = create_service<arm_msgs::srv::MoveToPose>(
            "arm/move_to_pose",
            [this](const std::shared_ptr<arm_msgs::srv::MoveToPose::Request> request,
                   std::shared_ptr<arm_msgs::srv::MoveToPose::Response> response) {
                on_move_to_pose(*request, *response);
            });

        // A tick, so that a classifier going quiet is noticed rather than
        // waited on forever.
        timer_ = create_wall_timer(100ms, [this]() {
            const MapResult result = mapper_.tick(now().seconds());
            if (result.outcome == MapOutcome::kStale && result.accepted_gesture == 0 &&
                last_published_ != 0) {
                publish_move(result.target);
                last_published_ = 0;
                RCLCPP_WARN(get_logger(), "no gesture recently enough; returning to rest");
            }
        });
    }

  private:
    static GestureMapConfig read_config(rclcpp::Node& node) {
        GestureMapConfig config;
        config.confidence_threshold =
            node.declare_parameter<double>("confidence_threshold", config.confidence_threshold);
        config.dwell_s = node.declare_parameter<double>("dwell_s", config.dwell_s);
        config.staleness_s = node.declare_parameter<double>("staleness_s", config.staleness_s);
        return config;
    }

    void on_gesture(const arm_msgs::msg::GestureCommand& message) {
        const MapResult result =
            mapper_.update(message.gesture_id, message.confidence, message.valid, now().seconds());
        if (result.outcome != MapOutcome::kAccepted) return;

        RCLCPP_INFO(get_logger(), "gesture %u accepted after %.0f ms of dwell",
                    static_cast<unsigned>(result.accepted_gesture),
                    result.dwell_elapsed_s * 1000.0);

        if (!result.moves_arm) {
            // A hand configuration this arm has no fingers for. Reported, not
            // acted on, so the terminal device can do something with it
            // without the arm pretending to.
            RCLCPP_INFO(get_logger(), "gesture %u does not move this mechanism",
                        static_cast<unsigned>(result.accepted_gesture));
            last_published_ = result.accepted_gesture;
            return;
        }

        publish_move(result.target);
        last_published_ = result.accepted_gesture;
    }

    void publish_move(const ak::JointVector& target) {
        const ak::JointVector start = have_state_ ? current_ : ak::JointVector{};
        const ak::JointTrajectory plan = ak::plan_joint_move(model_, start, target);
        auto message = to_message(joint_names_, plan, samples_);
        message.header.stamp = now();
        trajectory_->publish(message);
    }

    void on_move_to_pose(const arm_msgs::srv::MoveToPose::Request& request,
                         arm_msgs::srv::MoveToPose::Response& response) {
        ak::Transform target;
        target.translation = {request.target.position.x, request.target.position.y,
                              request.target.position.z};
        const double w = request.target.orientation.w;
        const double x = request.target.orientation.x;
        const double y = request.target.orientation.y;
        const double z = request.target.orientation.z;
        const double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm < 1e-9) {
            response.accepted = false;
            response.message = "the requested orientation is not a rotation";
            return;
        }
        const double nw = w / norm, nx = x / norm, ny = y / norm, nz = z / norm;
        target.rotation(0, 0) = 1.0 - 2.0 * (ny * ny + nz * nz);
        target.rotation(0, 1) = 2.0 * (nx * ny - nz * nw);
        target.rotation(0, 2) = 2.0 * (nx * nz + ny * nw);
        target.rotation(1, 0) = 2.0 * (nx * ny + nz * nw);
        target.rotation(1, 1) = 1.0 - 2.0 * (nx * nx + nz * nz);
        target.rotation(1, 2) = 2.0 * (ny * nz - nx * nw);
        target.rotation(2, 0) = 2.0 * (nx * nz - ny * nw);
        target.rotation(2, 1) = 2.0 * (ny * nz + nx * nw);
        target.rotation(2, 2) = 1.0 - 2.0 * (nx * nx + ny * ny);

        const ak::JointVector start = have_state_ ? current_ : ak::JointVector{};

        if (request.cartesian_path) {
            const double duration = request.duration > 0.0 ? request.duration : 2.0;
            const ak::CartesianPlan plan =
                ak::plan_cartesian_line(model_, start, target, duration, samples_);
            if (plan.status != ak::IkStatus::kSuccess) {
                response.accepted = false;
                response.message = plan.failed_at >= 0
                                       ? "the straight-line path leaves the workspace partway along"
                                       : "the path could not be planned";
                return;
            }
            trajectory_msgs::msg::JointTrajectory message;
            message.header.stamp = now();
            message.joint_names = joint_names_;
            for (const auto& point : plan.points) {
                trajectory_msgs::msg::JointTrajectoryPoint out;
                out.positions.assign(point.q.begin(), point.q.end());
                out.velocities.assign(point.qd.begin(), point.qd.end());
                out.time_from_start = rclcpp::Duration::from_seconds(point.time);
                message.points.push_back(out);
            }
            trajectory_->publish(message);
            response.accepted = true;
            response.planned_duration = plan.duration;
            response.message = "straight-line path planned";
            return;
        }

        const ak::IkResult solution = ak::inverse_kinematics(model_, target, start);
        response.position_error = solution.position_error;
        response.orientation_error = solution.orientation_error;
        if (!solution.ok()) {
            response.accepted = false;
            response.message = solution.status == ak::IkStatus::kUnreachable
                                   ? "the pose is outside the arm's reach"
                                   : "the pose needs a joint past its stop";
            return;
        }

        const ak::JointTrajectory plan = ak::plan_joint_move(model_, start, solution.q);
        auto message = to_message(joint_names_, plan, samples_);
        message.header.stamp = now();
        trajectory_->publish(message);

        response.accepted = true;
        response.planned_duration = plan.duration();
        response.message = "planned";
    }

    ak::ArmModel model_;
    GestureMapper mapper_;
    std::vector<std::string> joint_names_;
    ak::JointVector current_{};
    bool have_state_{false};
    int samples_{40};
    int last_published_{-1};

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_;
    rclcpp::Subscription<arm_msgs::msg::GestureCommand>::SharedPtr gesture_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
    rclcpp::Service<arm_msgs::srv::MoveToPose>::SharedPtr move_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace arm_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<arm_control::GestureMapperNode>());
    rclcpp::shutdown();
    return 0;
}
