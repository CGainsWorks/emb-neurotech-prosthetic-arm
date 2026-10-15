// Inverse kinematics: a closed-form solver for the A1 geometry and a damped
// least-squares solver for anything.
//
// Both are here on purpose. The analytic solver is exact, returns every branch
// and costs about a microsecond, so it is what the controller runs. The
// numerical one makes no assumption about the DH table, so it still works if
// the wrist is ever built without concurrent axes and it is the independent
// implementation the analytic solver is tested against. Agreement between two
// solvers written from different principles is worth more than either one
// agreeing with itself.
#pragma once

#include <vector>

#include "arm_kinematics/forward_kinematics.hpp"
#include "arm_kinematics/model.hpp"

namespace ak {

// How a branch of the closed-form solution was chosen, kept alongside the
// angles so a caller can prefer, say, elbow-up without re-deriving it.
struct IkBranch {
    bool shoulder_forward{true};
    bool elbow_up{true};
    bool wrist_flipped{false};
};

struct IkSolution {
    JointVector q{};
    IkBranch branch{};
    bool within_limits{true};
};

// The metric the branch selector minimises: a weighted sum of squared joint
// displacements from the seed, with the proximal joints weighted heaviest
// because they swing the most mass. Exposed so a caller that wants to rank the
// branches itself ranks them the same way the solver does and so a test can
// check the choice without restating the weights.
double branch_seed_cost(const JointVector& candidate, const JointVector& seed);

// Every closed-form solution for the pose, in-limits or not. Empty when the
// pose is out of reach. At most eight, fewer when branches coincide.
std::vector<IkSolution> inverse_kinematics_all(const ArmModel& model, const Transform& pose,
                                               const JointVector& seed);

enum class IkStatus {
    kSuccess,
    kUnreachable,     // the wrist centre lies outside the shell the arm can reach
    kOutsideLimits,   // solutions exist but every one violates a joint stop
    kNoConvergence,   // numerical solver only
};

struct IkResult {
    JointVector q{};
    IkStatus status{IkStatus::kUnreachable};
    IkBranch branch{};
    int iterations{0};        // numerical solver only
    double position_error{0.0};     // m
    double orientation_error{0.0};  // rad
    bool ok() const { return status == IkStatus::kSuccess; }
};

// The in-limits closed-form solution closest to the seed, weighted so that the
// large proximal joints move as little as possible.
IkResult inverse_kinematics(const ArmModel& model, const Transform& pose,
                            const JointVector& seed);

struct NumericalIkOptions {
    int max_iterations{200};
    double position_tolerance{1e-9};      // m
    double orientation_tolerance{1e-9};   // rad
    double damping{1e-3};                 // base lambda for the damped inverse
    double singularity_threshold{1e-3};   // below this, damping is raised
    double max_step{0.35};                // rad, per joint per iteration
    bool respect_limits{true};
};

IkResult inverse_kinematics_numerical(const ArmModel& model, const Transform& pose,
                                      const JointVector& seed,
                                      const NumericalIkOptions& options = {});

}  // namespace ak
