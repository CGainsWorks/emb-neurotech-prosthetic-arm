#include "arm_control/joint_controller.hpp"

#include <algorithm>
#include <cmath>

namespace arm_control {

void LoopTiming::reset() {
    iterations_ = 0;
    overruns_ = 0;
    last_s_ = 0.0;
    total_period_s_ = 0.0;
    max_period_s_ = 0.0;
    min_period_s_ = 1e9;
    squared_error_s_ = 0.0;
    intervals_ = 0;
    started_ = false;
}

void LoopTiming::record(double now_s, double nominal_period_s, double tolerance_ratio) {
    ++iterations_;
    if (!started_) {
        started_ = true;
        last_s_ = now_s;
        return;
    }

    const double period = now_s - last_s_;
    last_s_ = now_s;
    if (period <= 0.0) return;

    ++intervals_;
    total_period_s_ += period;
    max_period_s_ = std::max(max_period_s_, period);
    min_period_s_ = std::min(min_period_s_, period);

    const double error = period - nominal_period_s;
    squared_error_s_ += error * error;
    if (error > nominal_period_s * tolerance_ratio) ++overruns_;
}

double LoopTiming::mean_period_ms() const {
    return intervals_ > 0 ? (total_period_s_ / intervals_) * 1000.0 : 0.0;
}

double LoopTiming::jitter_rms_ms() const {
    return intervals_ > 0 ? std::sqrt(squared_error_s_ / intervals_) * 1000.0 : 0.0;
}

JointController::JointController(const ak::ArmModel& model, const ControllerConfig& config)
    : model_(model), config_(config) {}

void JointController::set_trajectory(const ak::JointTrajectory& trajectory, double start_s) {
    trajectory_ = trajectory;
    trajectory_start_s_ = start_s;
    active_ = !trajectory.empty();
}

void JointController::hold() { active_ = false; }

ControlCommand JointController::update(double now_s, const ak::JointVector& measured_position,
                                       const ak::JointVector& measured_velocity) {
    const double nominal = config_.rate_hz > 0.0 ? 1.0 / config_.rate_hz : 0.0;
    timing_.record(now_s, nominal, config_.overrun_tolerance_ratio);

    ControlCommand command;

    if (active_) {
        const double elapsed = now_s - trajectory_start_s_;
        const ak::TrajectoryPoint point = trajectory_.sample(elapsed);
        reference_ = point.q;
        command.velocity = point.qd;
        command.trajectory_active = true;
        if (elapsed >= trajectory_.duration()) {
            active_ = false;
            command.trajectory_finished = true;
        }
    }

    command.reference = reference_;

    // Gravity at the reference, not at the measurement. Using the measurement
    // closes a loop through the compensation itself and with a compliant
    // servo underneath it that loop is not obviously stable. The reference is
    // where the arm is going, which is what the compensation should be sized
    // for anyway.
    command.gravity_torque = ak::gravity_torque(model_, reference_);

    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        const double gravity_offset = command.gravity_torque[i] * config_.compliance_rad_per_nm;
        const double lead_offset = command.velocity[i] * config_.velocity_lead_s;

        const double gravity_capped =
            std::max(-config_.max_compensation_rad,
                     std::min(config_.max_compensation_rad, gravity_offset));
        const double lead_capped =
            std::max(-config_.max_compensation_rad,
                     std::min(config_.max_compensation_rad, lead_offset));

        command.compensation[i] = gravity_capped;
        command.lead[i] = lead_capped;
        command.position[i] = reference_[i] + gravity_capped + lead_capped;
    }

    // The command must be inside the stops whatever the compensation asked
    // for. The firmware clamps as well; doing it here too means the telemetry
    // reports what was actually asked for rather than what was sent.
    ak::clamp_to_limits(model_, command.position);

    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        peak_error_[i] = std::max(peak_error_[i], std::fabs(reference_[i] - measured_position[i]));
    }
    (void)measured_velocity;

    return command;
}

}  // namespace arm_control
