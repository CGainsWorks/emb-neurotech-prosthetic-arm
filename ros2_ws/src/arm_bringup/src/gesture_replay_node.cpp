// A scripted classifier.
//
// Publishes gesture decisions at the rate the real one does, following a
// sequence from parameters, with a configurable rate of confident wrong
// answers mixed in. It exists so the control stack can be exercised, in RViz
// or in a launch test, without ml-semg-gesture-classifier and without a
// wearer. It is not a classifier and does not look at the sEMG at all.
#include <chrono>
#include <memory>
#include <vector>

#include "arm_control/gesture_map.hpp"
#include "arm_msgs/msg/gesture_command.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {

class GestureReplay : public rclcpp::Node {
  public:
    GestureReplay() : rclcpp::Node("gesture_replay") {
        const auto sequence = declare_parameter<std::vector<int64_t>>(
            "sequence", std::vector<int64_t>{0, 13, 0, 10, 9, 6, 14, 0});
        hold_s_ = declare_parameter<double>("hold_s", 2.0);
        confidence_ = declare_parameter<double>("confidence", 0.88);
        misclassify_every_ = declare_parameter<int>("misclassify_every", 7);
        rate_hz_ = declare_parameter<double>("rate_hz", 20.0);

        for (int64_t value : sequence) sequence_.push_back(static_cast<uint8_t>(value));
        if (sequence_.empty()) sequence_.push_back(0);

        publisher_ = create_publisher<arm_msgs::msg::GestureCommand>("gesture/command", 20);

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]() { tick(); });

        RCLCPP_WARN(get_logger(),
                    "publishing a scripted gesture sequence; no classifier and no sEMG are "
                    "involved");
        poses_ = arm_control::default_gesture_poses();
    }

  private:
    void tick() {
        const double elapsed = (now() - start_).seconds();
        const auto index = static_cast<std::size_t>(elapsed / hold_s_) % sequence_.size();
        const uint8_t intended = sequence_[index];

        arm_msgs::msg::GestureCommand message;
        message.header.stamp = now();
        message.gesture_id = intended;
        message.confidence = static_cast<float>(confidence_);
        message.valid = true;
        // A stand-in for the classifier's own latency: the window it decided
        // from ended 200 ms ago plus whatever inference cost. See
        // docs/CONTROL_LOOP_TIMING.md.
        message.inference_latency_ms = 0.3f;

        ++published_;
        if (misclassify_every_ > 0 && intended != 0 && published_ % misclassify_every_ == 0) {
            message.gesture_id = static_cast<uint8_t>((intended % 17) + 1);
            message.confidence = 0.9f;
        }

        for (const auto& pose : poses_) {
            if (pose.id == message.gesture_id) {
                message.gesture_name = pose.name;
                break;
            }
        }
        publisher_->publish(message);
    }

    std::vector<uint8_t> sequence_;
    std::vector<arm_control::GesturePose> poses_;
    double hold_s_{2.0};
    double confidence_{0.88};
    double rate_hz_{20.0};
    int misclassify_every_{7};
    uint64_t published_{0};
    rclcpp::Time start_{now()};
    rclcpp::Publisher<arm_msgs::msg::GestureCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GestureReplay>());
    rclcpp::shutdown();
    return 0;
}
