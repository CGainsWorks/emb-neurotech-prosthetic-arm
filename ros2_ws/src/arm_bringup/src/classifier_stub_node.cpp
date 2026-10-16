// A stand-in for the gesture classifier, behind its real interface.
//
// ml-semg-gesture-classifier consumes sEMG frames and publishes a
// GestureCommand every 50 ms. This node does the same, on the same topics and
// at the same rate, so that the control stack downstream of it is driven by
// the sEMG rather than by a script. It is not a classifier and nothing about
// classification follows from it working.
//
// What it does: average each channel's envelope over the last 200 ms (the
// classifier's window), call the window rest if no channel is far above its
// floor and otherwise pick the candidate gesture whose electrode pattern is
// closest by cosine similarity, with a softmax over the similarities as the
// confidence. The patterns are arm_interface::activation_for(), which is also
// what the simulated board and the synthetic replay are built from, so on
// that signal it is right by construction. That is the point: it lets the
// end-to-end test check the plumbing from electrodes to joints without
// pretending to check a decision.
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "arm_control/gesture_map.hpp"
#include "arm_interface/simulated_device.hpp"
#include "arm_msgs/msg/gesture_command.hpp"
#include "arm_msgs/msg/semg_frame.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

constexpr std::size_t kChannels = 8;

class ClassifierStub : public rclcpp::Node {
  public:
    ClassifierStub() : rclcpp::Node("classifier_stub") {
        // Gestures 6 and 14 recruit the same electrodes in activation_for(),
        // so only one of them can be a candidate at a time.
        const auto candidates = declare_parameter<std::vector<int64_t>>(
            "candidates", std::vector<int64_t>{9, 10, 13, 6});
        window_frames_ = static_cast<std::size_t>(declare_parameter<int>("window_frames", 40));
        rest_counts_ = declare_parameter<double>("rest_counts", 60.0);
        active_counts_ = declare_parameter<double>("active_counts", 2200.0);
        rest_fraction_ = declare_parameter<double>("rest_fraction", 0.15);
        sharpness_ = declare_parameter<double>("sharpness", 12.0);
        stale_s_ = declare_parameter<double>("stale_s", 0.5);
        const double rate_hz = declare_parameter<double>("rate_hz", 20.0);

        for (int64_t id : candidates) {
            Template entry;
            entry.id = static_cast<uint8_t>(id);
            const auto pattern = arm_interface::activation_for(entry.id);
            double norm = 0.0;
            for (std::size_t i = 0; i < kChannels; ++i) norm += pattern.weights[i] * pattern.weights[i];
            norm = std::sqrt(norm);
            for (std::size_t i = 0; i < kChannels; ++i)
                entry.weights[i] = norm > 0.0 ? pattern.weights[i] / norm : 0.0;
            templates_.push_back(entry);
        }
        for (const auto& pose : arm_control::default_gesture_poses()) names_[pose.id] = pose.name;

        frames_ = create_subscription<arm_msgs::msg::SemgFrame>(
            "semg/frame", 50, [this](arm_msgs::msg::SemgFrame::SharedPtr message) {
                std::array<double, kChannels> envelopes{};
                for (std::size_t i = 0; i < kChannels; ++i) envelopes[i] = message->envelopes[i];
                window_.push_back(envelopes);
                while (window_.size() > window_frames_) window_.pop_front();
                last_frame_ = now();
                have_frame_ = true;
            });
        publisher_ = create_publisher<arm_msgs::msg::GestureCommand>("gesture/command", 20);

        const auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]() { decide(); });

        RCLCPP_WARN(get_logger(),
                    "classifier interface stub: nearest-pattern decisions, not a trained "
                    "classifier");
    }

  private:
    struct Template {
        uint8_t id{0};
        std::array<double, kChannels> weights{};
    };

    void decide() {
        const auto started = std::chrono::steady_clock::now();

        arm_msgs::msg::GestureCommand message;
        message.header.stamp = now();

        const bool fresh = have_frame_ && (now() - last_frame_).seconds() < stale_s_;
        if (!fresh || window_.size() < window_frames_) {
            // No signal, or not a full window of it: no decision, which the
            // mapper treats differently from a low-confidence one.
            message.valid = false;
            publisher_->publish(message);
            return;
        }

        std::array<double, kChannels> excess{};
        double peak = 0.0;
        double norm = 0.0;
        for (std::size_t i = 0; i < kChannels; ++i) {
            double mean = 0.0;
            for (const auto& frame : window_) mean += frame[i];
            mean /= static_cast<double>(window_.size());
            excess[i] = std::max(0.0, (mean - rest_counts_) / (active_counts_ - rest_counts_));
            peak = std::max(peak, excess[i]);
            norm += excess[i] * excess[i];
        }
        norm = std::sqrt(norm);

        if (peak < rest_fraction_ || norm <= 0.0) {
            message.gesture_id = 0;
            message.confidence = static_cast<float>(1.0 - 0.5 * peak / rest_fraction_);
        } else {
            std::vector<double> scores;
            double total = 0.0;
            for (const auto& entry : templates_) {
                double similarity = 0.0;
                for (std::size_t i = 0; i < kChannels; ++i)
                    similarity += entry.weights[i] * excess[i] / norm;
                const double score = std::exp(sharpness_ * similarity);
                scores.push_back(score);
                total += score;
            }
            std::size_t best = 0;
            for (std::size_t i = 1; i < scores.size(); ++i)
                if (scores[i] > scores[best]) best = i;
            message.gesture_id = templates_[best].id;
            message.confidence = static_cast<float>(scores[best] / total);
        }
        message.valid = true;
        const auto it = names_.find(message.gesture_id);
        if (it != names_.end()) message.gesture_name = it->second;
        message.inference_latency_ms = static_cast<float>(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
                .count());
        publisher_->publish(message);
    }

    std::vector<Template> templates_;
    std::map<uint8_t, std::string> names_;
    std::deque<std::array<double, kChannels>> window_;
    std::size_t window_frames_{40};
    double rest_counts_{60.0};
    double active_counts_{2200.0};
    double rest_fraction_{0.15};
    double sharpness_{12.0};
    double stale_s_{0.5};
    bool have_frame_{false};
    rclcpp::Time last_frame_;
    rclcpp::Subscription<arm_msgs::msg::SemgFrame>::SharedPtr frames_;
    rclcpp::Publisher<arm_msgs::msg::GestureCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClassifierStub>());
    rclcpp::shutdown();
    return 0;
}
