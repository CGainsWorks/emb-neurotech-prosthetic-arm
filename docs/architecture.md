# System architecture

Acquisition through to actuation and which part of this repository owns each
stage.

```
  electrodes            THE BOARD (designed, not built)
      │                 ┌──────────────────────────────────────────┐
      ├─ 8 differential │  INA333 ×8 ─ HPF ─ ×30 ─ 477 Hz LPF       │
      │  pairs          │       │                                   │
      │                 │   ADC1, 16 bit, 2 kHz, DMA                │
      │                 │       │                                   │
      │                 │   DC block ─ 50 Hz notch ─ 80 Hz LPF ─ ÷10│
      │                 │       │                        │          │
      │                 │       │                   envelope (4 Hz) │
      │                 │   200 Hz frames                           │
      │                 │       │                                   │
      │                 │   LSM6DSOX ── complementary filter        │
      │                 │       │                                   │
      │                 │   supervisor ── servo PWM ×6 ── servos    │
      │                 │       │                                   │
      │                 │   COBS + CRC-16 framing                   │
      │                 └───────┬──────────────────────────────────┘
      │                         │ USART1, 921600 baud
      │                 ┌───────┴──────────────────────────────────┐
      │                 │  arm_interface: firmware_bridge          │
      │                 │       │                                   │
      │                 │  /semg/frame  /arm/telemetry  /arm/imu   │
      │                 └───────┬──────────────────────────────────┘
      │                         │
      │   ml-semg-gesture-classifier (a separate repository)
      │                         │ /gesture/command
      │                 ┌───────┴──────────────────────────────────┐
      │                 │  arm_control: gesture_mapper             │
      │                 │   confidence gate ─ dwell ─ pose table   │
      │                 │   trajectory planner (arm_kinematics)    │
      │                 │       │ /arm/trajectory                   │
      │                 │  arm_control: joint_controller, 100 Hz   │
      │                 │   sample ─ gravity feedforward ─ lead    │
      │                 │       │ /arm/joint_command                │
      │                 └───────┴──────────────────────────────────┘
      │                         │
      └─────────────────────────┘  back to the bridge and the board
```

## The five pieces and why they are separate

### `kinematics/`: the maths, with no dependencies

Forward and inverse kinematics, rigid-body dynamics, trajectory generation.
Plain C++17, no ROS, no Eigen, no third-party anything: it builds with
`./kinematics/run_tests.sh` on a machine with nothing but a compiler.

It is also the single source of truth for the mechanism. The URDF, the model
JSON, the actuator ratings in the thermal budget and the joint limits in the
firmware all derive from
[`kinematics/src/model.cpp`](../kinematics/src/model.cpp) and CI regenerates
them and fails on any difference.

Dependency-free because it is compiled into three different things: the host
tests, the ROS 2 nodes and the simulated board. A dependency on the critical
path of all three would have to be worth something and at 6×6 matrices Eigen
is not.

### `firmware/`: the board, split in two

`firmware/src/` is portable C++ with no register access: signal conditioning,
servo limiting, the IMU register driver, the wire protocol, the fault
supervisor and the scheduler. It is compiled into the host test suite, where
it is tested and into the ROS bridge, which speaks the link by running the
same encoder the board runs.

`firmware/src/platform/stm32/` is everything that touches a register. It is
compiled only by the cross build and it is the part that cannot be tested
until a board exists.

That split is the whole structure of the firmware. Everything with judgement
in it is on the testable side; what is left is wiring.

### `ros2_ws/src/arm_interface`: the bridge and a board that is not there

`firmware_bridge` owns the link. Behind it is a `DeviceLink`: either a POSIX
serial port, which has never been connected to anything, or a `SimulatedLink`
that runs the board in the same process.

The simulator is why "tested in simulation" means something here. It takes the
same bytes the firmware takes, decodes them with the firmware's own decoder,
runs the arm through the same rigid-body dynamics the planner uses and
encodes telemetry back with the firmware's own encoder. A mistake in the
framing, the field order, the endianness or a message length fails there
exactly as it would on a cable.

What it does not simulate is the analogue front end, the electrodes or the
physical arm.

### `ros2_ws/src/arm_control`: the decisions

`gesture_mapper` turns a class into a pose, with the gating that makes a noisy
classifier usable: a confidence floor, a dwell and rest privileged so that
stopping is never delayed.

`joint_controller` runs at 100 Hz, samples the trajectory and adds two
corrections the servos cannot make for themselves, because there is no
output-shaft encoder anywhere on this arm: a gravity feedforward through the
servo's own compliance and a velocity lead through its own lag.

Both are thin nodes over a library with unit tests. The judgement is in
`arm_control_core`; the nodes are subscriptions and publishers.

### `hardware/semg-servo-board/`: a design and its arithmetic

No schematic, no layout, no board. What exists is a parameterised model of the
analogue front end and the power tree, the documents that quote it and the
tests that check the documents still agree with the model and that the model
still agrees with the firmware constants derived from it.

## Three decisions worth arguing about

### Position servos and what that forces

The actuators take a position and close their own loop. That removes any
possibility of torque control, impedance control or collision detection and
it is why the controller's job is choosing which position to send rather than
which torque to apply.

It also means the arm has no idea where it actually is. The telemetry reports
the commanded position because that is all the board knows. In simulation the
true position is available and is what the tracking error is measured against,
which is why that number means something; on hardware it would not exist.

The alternative is actuators with encoders and a current loop, which is a
different arm at several times the cost. The honest statement is that this is
a cheap mechanism and the control stack is built around that rather than
pretending otherwise.

### The classifier runs on the host

The board ships frames and the host decides. It could be the other way round:
the classifier's own estimate puts its deployed int8 network at 13 ms on this
microcontroller, which fits.

It is on the host because the board has nothing to gain from it. The link
carries 22 kB/s against a 92 kB/s line, the decision has to reach a host
anyway for anything useful to happen with it and a model that lives on a host
can be retrained for a wearer in an afternoon rather than reflashed.

A wearable version would move it and the part was chosen so that it can.

### One repository for hardware, firmware and ROS

They are one system and the interfaces between them are where the mistakes
live. The gain constant the firmware uses to turn counts into microvolts is
the analogue design's gain; the joint limits the firmware clamps to are the
kinematic model's; the wire protocol the bridge speaks is the firmware's own
source file. Each of those is checked by a test that reads both sides.

Split across three repositories, each of those checks becomes a version
number and a hope.

## What is not here

- **No ros2_control.** The hardware interface abstraction buys portability
  between controllers this arm will never use and costs a dependency and a
  layer of indirection between the trajectory and the serial port. The
  controller is 200 lines.
- **No Gazebo.** The simulated board integrates the same dynamics the planner
  uses, which is what the control stack needs to be tested against. A physics
  engine would add contact and friction models that nothing here is calibrated
  for and that would not be evidence about anything.
- **No MoveIt.** The planner is point-to-point in joint space with a straight
  line in Cartesian space when asked for one. There is no obstacle field and
  no inverse kinematics problem harder than a closed form.
- **No measured anything.** Repeated throughout and in
  [STATUS.md](../STATUS.md) at the top.
