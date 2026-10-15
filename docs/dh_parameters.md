# Kinematic derivation

The A1 is an anthropomorphic shoulder-elbow chain closed by a roll-pitch-roll
wrist: six revolute joints, with the last three axes made concurrent.

Everything below is generated from, or checked against,
[`kinematics/src/model.cpp`](../kinematics/src/model.cpp), which is the single
source of truth for the mechanism. The URDF in
[`arm_description`](../ros2_ws/src/arm_description) is generated from it and CI
fails if the two disagree.

No arm has been manufactured. These are design dimensions.

## The table

Standard (distal) Denavit-Hartenberg, so that

```
A_i = Rz(theta_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)
```

| i | Joint | theta | d (m) | a (m) | alpha | Range |
|---|---|---|---|---|---|---|
| 1 | shoulder rotation | q1 | 0.110 | 0 | +90° | ±100° |
| 2 | shoulder flexion | q2 | 0 | 0.300 | 0 | ±120° |
| 3 | elbow flexion | q3 | 0 | 0 | +90° | ±145° |
| 4 | forearm rotation | q4 | 0.260 | 0 | −90° | ±175° |
| 5 | wrist flexion | q5 | 0 | 0 | +90° | ±100° |
| 6 | tool roll | q6 | 0.095 | 0 | 0 | ±175° |

Standard rather than modified DH because the closed-form solver below is the
textbook anthropomorphic decomposition, which is written in this convention.
Translating it would add a transcription risk for no gain.

### Where the dimensions come from

| | | |
|---|---|---|
| d1 = 110 mm | base plate to the shoulder pitch axis | the bench mount, not anatomy |
| a2 = 300 mm | upper arm | adult male acromion to radiale, rounded |
| d4 = 260 mm | forearm | adult male radiale to stylion, rounded |
| d6 = 95 mm | wrist centre to the terminal device reference | the depth of the terminal device |

Total reach from the base is 765 mm and the sampled workspace reaches 764 mm
of it, which is as close as uniform joint sampling gets to a fully extended
arm.

### The home pose

At q = 0 the shoulder lifts the chain to d1, the upper arm reaches a2 along
the base x axis and the forearm and wrist hang straight down by d4 + d6. The
tool sits at (0.300, 0, −0.245) with its approach axis pointing at the bench.

That is the one pose that can be read off the table by hand, which is why
`the_home_pose_is_the_arm_out_and_the_forearm_down` in
[`test_forward_kinematics.cpp`](../kinematics/test/test_forward_kinematics.cpp)
anchors everything else to it.

## The wrist is concurrent on purpose

Joints 4, 5 and 6 intersect at a point, because a4 = a5 = 0 and d5 = 0.

That costs anatomical fidelity: a human wrist's flexion and deviation axes do
not intersect and this arm has no deviation at all. What it buys is a closed
form. With a concurrent wrist the problem separates: the wrist centre depends
only on the first three joints, so position and orientation can be solved one
after the other instead of together.

The consequence is in the numbers.
[`analysis/kinematics_latency.json`](../analysis/kinematics_latency.json) has
the closed-form solver at 5.5 microseconds median against the damped
least-squares solver's 74 and the gap at the tail is far larger: 22
microseconds at the ninety-ninth percentile against 10.9 milliseconds, because
the iterative solver's worst case is running out of iterations. It also
converges from a neutral seed only three times in four, where the closed form
returns every branch, every time.

`the_wrist_centre_does_not_move_when_the_wrist_turns` asserts the property the
whole derivation rests on.

## Forward kinematics

Composing the six transforms gives, for the wrist centre pc = p − d6·a where a
is the approach axis:

```
pc_x = cos(q1) (a2 cos(q2) + d4 sin(q2 + q3))
pc_y = sin(q1) (a2 cos(q2) + d4 sin(q2 + q3))
pc_z = d1 + a2 sin(q2) − d4 cos(q2 + q3)
```

and the orientation of frame 3 in the base:

```
          [ c1 c23    s1   c1 s23 ]
R_03  =   [ s1 c23   −c1   s1 s23 ]
          [   s23      0    −c23  ]
```

## Inverse kinematics, in closed form

### Joint 1

```
q1 = atan2(pc_y, pc_x)        or        q1 + pi
```

Two branches: the arm reaching forwards and the arm reaching back over itself.
When pc lies on the shoulder axis the planar radius vanishes, joint 1 does not
affect the wrist centre at all and the solver keeps the seed value rather than
inventing one.

### Joint 3

With r the signed planar radius (positive for the forward branch, negative for
the backward one) and s = pc_z − d1:

```
r² + s² = a2² + d4² + 2 a2 d4 sin(q3)
```

so

```
sin(q3) = (r² + s² − a2² − d4²) / (2 a2 d4)
cos(q3) = ± sqrt(1 − sin²(q3))
```

The sign is the elbow-up and elbow-down branch. A value of sin(q3) outside
[−1, 1] is a wrist centre outside the shell the arm can reach and the solver
reports that rather than clamping to the nearest reachable pose.

### Joint 2

With A = a2 + d4 sin(q3) and B = d4 cos(q3),

```
r = A cos(q2) + B sin(q2)
s = A sin(q2) − B cos(q2)
```

which inverts directly:

```
q2 = atan2(B r + A s, A r − B s)
```

### Joints 4, 5 and 6

With the arm placed, whatever rotation is left over belongs to the wrist:

```
R_36 = R_03ᵀ R_desired
```

Expanding the product of the last three DH rotations:

```
            [ c4 c5 c6 − s4 s6   −c4 c5 s6 − s4 c6   c4 s5 ]
R_36    =   [ s4 c5 c6 + c4 s6   −s4 c5 s6 + c4 c6   s4 s5 ]
            [     −s5 c6              s5 s6            c5  ]
```

which reads off as

```
q5 = atan2(± sqrt(R02² + R12²), R22)
q4 = atan2(R12 / s5, R02 / s5)
q6 = atan2(R21 / s5, −R20 / s5)
```

The sign on s5 is the wrist-flip branch. When s5 vanishes, joints 4 and 6 are
collinear and only their sum is determined; the solver holds joint 4 at its
seed and puts the whole rotation into joint 6, which is the lighter of the two.
That is a choice, not a derivation and
`the_analytic_solver_handles_the_wrist_singularity_without_a_nan` pins it down.

### Eight branches

Two shoulder, two elbow, two wrist. `inverse_kinematics_all` returns all of
them, in or out of limits and `inverse_kinematics` picks the in-limits one
nearest the seed under a weighted metric that charges more for moving the
shoulder than the wrist.

Over 200,000 uniformly sampled configurations every pose is solved, with a
maximum position error of 6.7 × 10⁻¹⁶ m and a maximum orientation error of
1.8 × 10⁻¹¹ rad and a mean of 2.9 of the eight branches inside the joint
limits. Those are machine-precision numbers because the solution is
exact; the figures are in
[`analysis/workspace.json`](../analysis/workspace.json).

## Singularities

| | Where | What is lost |
|---|---|---|
| Shoulder | wrist centre on the base z axis | joint 1 has no effect on position |
| Elbow | q3 = ±90°, arm straight | motion along the arm's own length |
| Wrist | q5 = 0, joints 4 and 6 collinear | one rotational degree of freedom |

The elbow case is reachable inside the joint limits, which is why the numerical
solver damps rather than inverting: `manipulability()` returns the smallest
singular value of the Jacobian and the damping rises as it falls.
`manipulability_collapses_at_the_elbow_singularity` checks that the measure
actually sees it.

## Dynamics

Recursive Newton-Euler, in the same frames. The three things worth saying
about the implementation are in
[`kinematics/src/dynamics.cpp`](../kinematics/src/dynamics.cpp):

- Gravity is carried as a base acceleration of −g, so weight and inertia go
  through one recursion rather than two.
- The terminal device is folded into link 6 by the parallel axis theorem
  rather than carried as a seventh body: the same answer with one less special
  case.
- The mass matrix is built column by column from the same recursion, which is
  slower than a composite rigid body algorithm and is six columns of a 6×6.

It is checked against the gradient of the potential energy, against the
kinetic energy computed from link velocities and by round-tripping forward
and inverse dynamics. Those are three independent routes to the same numbers
and they agree to 1 × 10⁻⁹.

## Inertial parameters

Links are modelled as cylinders, with the actuator that drives the next joint
included in each link's mass.

| Link | Mass (kg) | Centre of mass in frame i (m) |
|---|---|---|
| 1 shoulder yoke | 0.35 | (0, 0, 0.020) |
| 2 upper arm | 0.55 | (−0.150, 0, 0) |
| 3 elbow housing | 0.25 | (0, 0, 0.030) |
| 4 forearm | 0.45 | (0, 0, −0.120) |
| 5 wrist pitch | 0.12 | (0, 0, 0) |
| 6 flange | 0.08 | (0, 0, −0.030) |
| terminal device | 0.40 | (0, 0, 0.035) in frame 6 |
| **total** | **2.20** | |

Frame i sits at the distal end of link i in the standard convention, which is
why most of the centres of mass are at a negative offset.

The worst-case gravity torque over the whole joint space is 5.0 N·m at the
shoulder flexion joint, against an 8 N·m actuator.
`the_worst_case_gravity_load_fits_inside_the_actuator_ratings` samples 200,000
configurations and asserts it for every joint, which is what connects this
table to
[`hardware/semg-servo-board/docs/thermal_budget.md`](../hardware/semg-servo-board/docs/thermal_budget.md).

2.2 kg is too heavy to wear. [`CLINICAL_LIMITATIONS.md`](CLINICAL_LIMITATIONS.md)
says so.
