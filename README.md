# emb-neurotech-prosthetic-arm

> **Status: planned. This repository is a brief, not an implementation.**
> Nothing here is built yet. The sections below define what goes in it.

The full prosthetic arm programme in one repository: custom hardware for sEMG acquisition and servo drive, embedded firmware, C++ forward and inverse kinematics and dynamics, and a ROS2 control stack tying signal acquisition through to actuation. Check the investor and patent position before making this public.

---

## What this covers

CV entries this repository is the evidence for:

- Six-axis myoelectric prosthetic arm, ROS2 control stack and C++ kinematics (NeuroTech Biowares)
- Custom STM32 sEMG PCB and 6-channel servo driver (NeuroTech Biowares)

---

## Requirements

### Hardware

- Six-axis arm with servo actuators
- sEMG electrodes and analogue front end with instrumentation amplifiers and filtering
- Custom STM32 PCB with 6-channel servo driver
- IMU on I2C
- Power regulation
- Battery
- Socket or mount

### Software

- Embedded C and C++ firmware
- sEMG sampling and filtering
- Servo PWM generation
- I2C IMU driver
- C++ forward and inverse kinematics and dynamics
- Trajectory planner
- ROS2 control stack in C++ and Python
- Real-time embedded runtime
- Classifier inference (see ml-semg-gesture-classifier)

### Report and documentation deliverables

- System architecture
- Kinematic derivation with DH parameters
- Schematic and signal-conditioning design with gain and bandwidth rationale
- Noise and artefact analysis
- Electrode interface notes
- Servo current and thermal budget
- Control loop timing budget
- Classification accuracy and latency
- User trial notes
- Safety statement for body-contact electronics
- Clinical limitations statement

---

## Inputs

- Differential sEMG electrode signals
- IMU orientation
- Joint encoder feedback
- Target grasp or Cartesian pose
- Gain and filter configuration

## Outputs

- Conditioned and digitised sEMG channels
- Classified gesture with confidence
- Joint angle trajectory
- Servo PWM drive commands
- Cartesian end-effector pose
- Telemetry and fault state

---

## Intended structure

```
hardware/
  semg-servo-board/
    kicad/ or altium/
    outputs/              gerbers, bom, schematic.pdf, 3d/
    docs/                 signal_conditioning.md, noise_analysis.md,
                          thermal_budget.md
firmware/
  src/                    semg_sampling, filtering, servo_pwm, imu_i2c
  include/
  test/
kinematics/               C++ FK, IK, dynamics library with unit tests
ros2_ws/src/
  arm_bringup/
  arm_description/        URDF, meshes
  arm_control/            trajectory planner, joint control
  arm_interface/          firmware bridge
  arm_msgs/
docs/                     architecture.md, dh_parameters.md, SAFETY.md,
                          CLINICAL_LIMITATIONS.md
```

Plus the standard baseline: `README.md`, `LICENSE`, `.gitignore`, `.gitattributes`
(Git LFS rules for weights, bags and media), `.github/workflows/ci.yml`, `docs/`,
`scripts/setup.sh`, `scripts/run_demo.sh` and `tests/`.

---

## Build checklist

- [ ] sEMG analogue front end design with a documented noise budget
- [ ] Servo driver stage with current and thermal budget
- [ ] PCB layout, fabrication and bring-up
- [ ] sEMG sampling firmware with measured noise floor
- [ ] Servo PWM generation and IMU I2C driver
- [ ] C++ forward kinematics with unit tests
- [ ] C++ inverse kinematics with unit tests
- [ ] Dynamics model
- [ ] URDF and ROS2 description
- [ ] ROS2 control stack: acquisition to actuation
- [ ] Trajectory planner
- [ ] Control loop timing budget measured
- [ ] Integration with the gesture classifier
- [ ] End-to-end demonstration and user trial notes

---

## Definition of done

This repository is finished when:

- [ ] The whole thing runs from one command on a clean machine.
- [ ] The README opens with a diagram or a GIF, not a wall of text.
- [ ] Every number quoted in the README is reproducible from a script in the repo.
- [ ] Limitations are stated honestly in their own section.
- [ ] A licence is declared.
- [ ] Large binaries live in Releases or LFS, not in the Git history.
- [ ] The repository URL is added to the matching CV bullet point.

---

## Starter prompt

Paste this into your coding agent of choice to begin.

<details>
<summary>Expand</summary>

```
You are helping me build a portfolio project from scratch. Read this brief and then
propose a concrete implementation plan before writing any code.

PROJECT: emb-neurotech-prosthetic-arm
The full prosthetic arm programme in one repository: custom hardware for sEMG acquisition and servo drive, embedded firmware, C++ forward and inverse kinematics and dynamics, and a ROS2 control stack tying signal acquisition through to actuation. Check the investor and patent position before making this public.

TARGET STACK
- Embedded C and C++ firmware
- sEMG sampling and filtering
- Servo PWM generation
- I2C IMU driver
- C++ forward and inverse kinematics and dynamics
- Trajectory planner
- ROS2 control stack in C++ and Python
- Real-time embedded runtime
- Classifier inference (see ml-semg-gesture-classifier)

HARDWARE CONTEXT
- Six-axis arm with servo actuators
- sEMG electrodes and analogue front end with instrumentation amplifiers and filtering
- Custom STM32 PCB with 6-channel servo driver
- IMU on I2C
- Power regulation
- Battery
- Socket or mount

THE SYSTEM MUST TAKE THESE INPUTS
- Differential sEMG electrode signals
- IMU orientation
- Joint encoder feedback
- Target grasp or Cartesian pose
- Gain and filter configuration

AND PRODUCE THESE OUTPUTS
- Conditioned and digitised sEMG channels
- Classified gesture with confidence
- Joint angle trajectory
- Servo PWM drive commands
- Cartesian end-effector pose
- Telemetry and fault state

INTENDED REPOSITORY LAYOUT
hardware/
  semg-servo-board/
    kicad/ or altium/
    outputs/              gerbers, bom, schematic.pdf, 3d/
    docs/                 signal_conditioning.md, noise_analysis.md,
                          thermal_budget.md
firmware/
  src/                    semg_sampling, filtering, servo_pwm, imu_i2c
  include/
  test/
kinematics/               C++ FK, IK, dynamics library with unit tests
ros2_ws/src/
  arm_bringup/
  arm_description/        URDF, meshes
  arm_control/            trajectory planner, joint control
  arm_interface/          firmware bridge
  arm_msgs/
docs/                     architecture.md, dh_parameters.md, SAFETY.md,
                          CLINICAL_LIMITATIONS.md

CONSTRAINTS
- Everything must run from a single command: `docker compose up` or `make demo`.
- No placeholder or stub code presented as working. If something is not implemented, say so.
- Every quantitative claim in the README must be reproducible from a script in this repo.
- Use only public or synthetic data. Record the licence of every dataset used.


START BY
1. Confirming or challenging the proposed repository layout.
2. Listing the specific libraries and versions you intend to pin.
3. Identifying the single riskiest technical unknown and how to de-risk it first.
Then work through the milestones in the README build checklist, one at a time.

```

</details>

---

## Notes

- Do not backdate commits to match CV dates. Publishing older work today is normal and expected.
- Record dataset licences before training on anything. Several common research datasets are
  non-commercial only.
- Flip this repository to public once there is something in it worth showing.
