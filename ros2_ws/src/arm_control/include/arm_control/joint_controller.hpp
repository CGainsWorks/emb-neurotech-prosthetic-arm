// The joint control loop: follow a trajectory, compensate for gravity and
// measure what the loop actually costs.
//
// The servos on this arm take a position and close their own loop around it,
// so this is not a torque controller. What it does is decide which position to
// send and it sends one that already accounts for the droop the servo will
// have under load: a position loop with no feedforward sits wherever the
// gravity torque and its own stiffness balance, which on the shoulder is
// several degrees away from where it was told. Feeding the gravity torque
// forward through the servo's own stiffness removes most of that without
// needing a torque sensor the arm does not have.
//
// The timing statistics are collected here rather than bolted on, because
// docs/CONTROL_LOOP_TIMING.md is only worth anything if the numbers in it came
// from the loop running.
#pragma once

#include <cstdint>
#include <vector>

#include "arm_kinematics/dynamics.hpp"
#include "arm_kinematics/model.hpp"
#include "arm_kinematics/trajectory.hpp"

namespace arm_control {

struct ControllerConfig {
    double rate_hz{100.0};

    // Radians of servo deflection per newton metre of load, per joint. This is
    // the reciprocal of the servo's positional stiffness and is the one number
    // in the controller that has to come from a measurement. It has not been
    // measured; the value is the design figure for the actuator class and
    // firmware/docs/BRING_UP.md has the procedure for replacing it with a real
    // one. Setting it to zero disables the compensation entirely, which is
    // what a cautious first power-up should do.
    double compliance_rad_per_nm{0.012};

    // Seconds of lead applied to the trajectory velocity.
    //
    // A servo that takes a position and closes its own loop lags when it is
    // moving: its loop is proportional plus derivative, so at constant speed it
    // settles wherever the derivative term balances the proportional one, which
    // is a following error proportional to velocity. On this arm that is about
    // seven degrees at a typical reach speed, which is enough to see.
    //
    // Nothing outside the servo can measure that error, because there is no
    // output-shaft encoder. What it can do is command a position the servo will
    // lag onto: leading by one lag time cancels the error to first order. The
    // lag time is the servo's own derivative-to-proportional gain ratio and is
    // measured at bring-up; firmware/docs/BRING_UP.md has the procedure. Zero
    // disables it.
    double velocity_lead_s{0.06};

    // Both corrections are capped, so that a modelling error cannot turn into a
    // large commanded offset.
    double max_compensation_rad{0.25};

    // A tick this much later than its slot is an overrun.
    double overrun_tolerance_ratio{0.25};
};

struct ControlCommand {
    ak::JointVector position{};       // what goes to the servos
    ak::JointVector reference{};      // where the trajectory wanted the arm
    ak::JointVector velocity{};       // trajectory velocity, for telemetry
    ak::JointVector gravity_torque{}; // the load being compensated
    ak::JointVector compensation{};   // gravity correction, after the cap
    ak::JointVector lead{};           // velocity correction, after the cap
    bool trajectory_active{false};
    bool trajectory_finished{false};
};

// Period, jitter and overruns, measured over the run.
class LoopTiming {
  public:
    void record(double now_s, double nominal_period_s, double tolerance_ratio);
    void reset();

    uint32_t iterations() const { return iterations_; }
    uint32_t overruns() const { return overruns_; }
    double mean_period_ms() const;
    double max_period_ms() const { return max_period_s_ * 1000.0; }
    double min_period_ms() const { return iterations_ > 1 ? min_period_s_ * 1000.0 : 0.0; }
    // Root mean square deviation from the nominal period, which is what
    // "jitter" means for a periodic loop.
    double jitter_rms_ms() const;

  private:
    uint32_t iterations_{0};
    uint32_t overruns_{0};
    double last_s_{0.0};
    double total_period_s_{0.0};
    double max_period_s_{0.0};
    double min_period_s_{1e9};
    double squared_error_s_{0.0};
    uint32_t intervals_{0};
    bool started_{false};
};

class JointController {
  public:
    JointController(const ak::ArmModel& model, const ControllerConfig& config);

    // Start following a new trajectory from `start_s`. Replaces whatever was
    // running: a new gesture supersedes the movement the last one started and
    // blending the two would trace a path neither of them asked for.
    void set_trajectory(const ak::JointTrajectory& trajectory, double start_s);

    // Hold the current reference. What the controller does when the classifier
    // has gone quiet.
    void hold();

    // One control tick.
    ControlCommand update(double now_s, const ak::JointVector& measured_position,
                          const ak::JointVector& measured_velocity);

    const LoopTiming& timing() const { return timing_; }
    void reset_timing() { timing_.reset(); }

    const ak::JointVector& reference() const { return reference_; }
    bool trajectory_active() const { return active_; }

    // Largest position error seen between the trajectory reference and the
    // measured position, per joint. The number that says whether the servo
    // model and the compensation are doing their job.
    const ak::JointVector& peak_tracking_error() const { return peak_error_; }

  private:
    ak::ArmModel model_;
    ControllerConfig config_;

    ak::JointTrajectory trajectory_{};
    double trajectory_start_s_{0.0};
    bool active_{false};

    ak::JointVector reference_{};
    ak::JointVector peak_error_{};
    LoopTiming timing_;
};

}  // namespace arm_control
