// sEMG replay.
//
// Publishes a stored sEMG recording on semg/frame at the board's 200 Hz frame
// rate, in place of a board. It is how the classifier path is exercised
// against a fixed signal: the same topic, message and rate the firmware bridge
// produces, from a file instead of a link. The bridge is told not to publish
// its own sEMG while this runs (publish_semg: false) so the two never
// interleave.
//
// The file the launch uses is synthetic (scripts/make_semg_replay.py) and
// contains no recording of anybody. Columns: eight raw samples, then eight
// envelopes, one row per frame, after a header row.
#include <array>
#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "arm_msgs/msg/semg_frame.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

struct Row {
    std::array<int16_t, 8> samples{};
    std::array<uint16_t, 8> envelopes{};
};

std::vector<Row> load(const std::string& path) {
    std::vector<Row> rows;
    std::ifstream in(path);
    std::string line;
    bool header = true;
    while (std::getline(in, line)) {
        if (header) {
            header = false;
            continue;
        }
        if (line.empty() || line[0] == '#') continue;
        std::stringstream fields(line);
        std::string field;
        Row row;
        for (std::size_t i = 0; i < 16 && std::getline(fields, field, ','); ++i) {
            const long value = std::stol(field);
            if (i < 8) {
                row.samples[i] = static_cast<int16_t>(value);
            } else {
                row.envelopes[i - 8] = static_cast<uint16_t>(value);
            }
        }
        rows.push_back(row);
    }
    return rows;
}

class SemgReplay : public rclcpp::Node {
  public:
    SemgReplay() : rclcpp::Node("semg_replay") {
        auto file = declare_parameter<std::string>("file", "");
        if (file.empty()) {
            file = ament_index_cpp::get_package_share_directory("arm_bringup") +
                   "/config/semg_replay.csv";
        }
        rate_hz_ = declare_parameter<double>("rate_hz", 200.0);
        start_delay_s_ = declare_parameter<double>("start_delay_s", 1.0);
        loop_ = declare_parameter<bool>("loop", false);

        rows_ = load(file);
        if (rows_.empty()) {
            RCLCPP_ERROR(get_logger(), "no frames in %s", file.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "replaying %zu frames (%.1f s) from %s", rows_.size(),
                        static_cast<double>(rows_.size()) / rate_hz_, file.c_str());
        }
        RCLCPP_WARN(get_logger(), "the sEMG is a replay of a synthetic file, not a wearer");

        publisher_ = create_publisher<arm_msgs::msg::SemgFrame>("semg/frame", 50);
        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]() { tick(); });
        start_ = now();
    }

  private:
    void tick() {
        if ((now() - start_).seconds() < start_delay_s_ || rows_.empty()) return;
        if (next_ >= rows_.size()) {
            if (!loop_) {
                if (!finished_) RCLCPP_INFO(get_logger(), "replay finished");
                finished_ = true;
                return;
            }
            next_ = 0;
        }
        const Row& row = rows_[next_++];

        arm_msgs::msg::SemgFrame message;
        message.header.stamp = now();
        message.header.frame_id = "forearm";
        message.sequence = sequence_;
        message.device_timestamp_us = static_cast<uint32_t>(
            static_cast<double>(sequence_) * 1e6 / rate_hz_);
        ++sequence_;
        for (std::size_t i = 0; i < 8; ++i) {
            message.samples[i] = row.samples[i];
            message.envelopes[i] = row.envelopes[i];
        }
        message.saturated_mask = 0;
        publisher_->publish(message);
    }

    std::vector<Row> rows_;
    std::size_t next_{0};
    uint32_t sequence_{0};
    double rate_hz_{200.0};
    double start_delay_s_{1.0};
    bool loop_{false};
    bool finished_{false};
    rclcpp::Time start_;
    rclcpp::Publisher<arm_msgs::msg::SemgFrame>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemgReplay>());
    rclcpp::shutdown();
    return 0;
}
