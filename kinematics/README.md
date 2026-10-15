# arm_kinematics

Forward and inverse kinematics, rigid-body dynamics and trajectory generation
for the A1 six-axis arm.

```bash
./run_tests.sh          # build and run everything; no ROS, no network
./run_tests.sh clean    # wipe the build directory first
```

73 tests, under a second. C++17 and nothing else: no ROS, no Eigen, no
third-party anything. It is compiled into three different consumers, so a
dependency on its critical path would have to earn its place three times
over and at 6×6 matrices Eigen does not.

## What is in it

| | |
|---|---|
| [`model.hpp`](include/arm_kinematics/model.hpp) | The mechanism: DH table, joint limits, inertial parameters. The single source of truth for the arm |
| [`forward_kinematics.hpp`](include/arm_kinematics/forward_kinematics.hpp) | Link transforms, tool pose, wrist centre, the geometric Jacobian and a manipulability measure from its smallest singular value |
| [`inverse_kinematics.hpp`](include/arm_kinematics/inverse_kinematics.hpp) | Two solvers: a closed form for this geometry and damped least squares for any geometry |
| [`dynamics.hpp`](include/arm_kinematics/dynamics.hpp) | Recursive Newton-Euler inverse dynamics, the mass matrix, forward dynamics and an integrator |
| [`trajectory.hpp`](include/arm_kinematics/trajectory.hpp) | Synchronised point-to-point moves with a quintic or trapezoidal profile and straight lines in Cartesian space |
| [`linalg.hpp`](include/arm_kinematics/linalg.hpp) | Fixed-size matrices, an LU solve with partial pivoting and a damped least-squares inverse |

[`docs/dh_parameters.md`](../docs/dh_parameters.md) has the derivation.

## Two inverse solvers and why both

The closed form is exact, returns all eight branches and runs in 5.5 µs. The
damped least-squares solver makes no assumption about the DH table at all.

Keeping both is not redundancy. They are written from different principles and
share nothing but the forward kinematics, so agreement between them over
thousands of random poses is evidence that neither is quietly wrong in some
corner of the workspace. `the_numerical_solver_agrees_with_the_analytic_one`
is that test.

The closed form is also the one that works. From a neutral seed the iterative
solver converges 76% of the time and its ninety-ninth percentile is 10.9 ms
against 22 µs, because its worst case is exhausting its iteration budget.

## How it is tested

The tests that matter are the ones that check a result against something
derived a different way, rather than against itself:

- **Gravity torque against the gradient of the potential energy**, by finite
  differences. A sign or a frame wrong anywhere in the Newton-Euler recursion
  fails this.
- **The mass matrix against the kinetic energy** computed from link
  velocities.
- **Forward dynamics round-tripped through inverse dynamics.**
- **Energy conservation to first order.** Halving the integrator's step has to
  halve the drift; a wrong Coriolis term would pump energy in at a rate that
  does not shrink with the step.
- **The Jacobian against numerical differentiation** of the forward
  kinematics.
- **Every closed-form branch reproducing the pose it was solved from**, to
  machine precision.
- **The worst-case gravity load against the actuator ratings**, sampled over
  200,000 configurations. That test is what connects this library to the
  thermal budget.

## Tools

Three command line tools, each printing JSON, because every number this
repository quotes comes out of a program rather than out of somebody's head.

```bash
./build/dump_model                           # the mechanism, as JSON
./build/workspace_report --samples 200000    # reach envelope and solver accuracy
./build/ik_benchmark --iterations 20000      # what everything costs, on this machine
```

`dump_model` is what [`scripts/generate_urdf.py`](../scripts/generate_urdf.py)
reads. CI regenerates the URDF from it and fails on any difference, so a link
length changed here either reaches the ROS description or stops the build.

## Using it elsewhere

It installs a CMake package:

```cmake
find_package(arm_kinematics REQUIRED)
target_link_libraries(your_target arm_kinematics::arm_kinematics)
```

and carries a `package.xml` with `<build_type>cmake</build_type>`, so colcon
builds it as a plain CMake package ahead of the ROS packages that consume it.
It lives outside `ros2_ws/src` because it does not depend on ROS and putting
it there would suggest otherwise;
[`scripts/build_ros.sh`](../scripts/build_ros.sh) passes colcon both base
paths.
