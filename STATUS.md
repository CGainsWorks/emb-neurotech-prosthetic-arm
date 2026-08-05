# Project status

Last updated: 5 August 2026

## Complete

- **A1 arm development kit PCB**, revision 1.1. Design archive and PDF in
  `hardware/arm-devkit/`. Imported from the `A1DevKit` repository.
- **sEMG device industrial design**, Shapr3D model and render in
  `hardware/semg-servo-board/`. Imported from the `OpenEMG` repository.

## Not started

- **sEMG analogue front end schematic.** No instrumentation amplifier stage,
  filtering or gain design exists as a schematic yet.
- **6-channel servo driver stage.** Not designed.
- **Firmware.** `firmware/` is empty. No sampling, filtering, servo PWM or IMU
  driver code.
- **Kinematics library.** `kinematics/` is empty. No forward kinematics, inverse
  kinematics or dynamics implementation, and no unit tests.
- **ROS2 control stack.** `ros2_ws/` is empty. No URDF, no description package,
  no control nodes.
- **Bring-up and validation.** No noise floor measurement, no thermal or current
  budget validation, no user trials.

## Before making this repository public

1. Confirm the investor and patent position for NeuroTech Biowares Ltd. If a
   filing is pending or an investor agreement restricts disclosure, this repo
   stays private.
2. Do not commit any personally recorded sEMG data. Subject recordings are
   personal data. Use a public dataset such as Ninapro for the classifier,
   which lives in `ml-semg-gesture-classifier`.
3. Add a clinical limitations statement. This is a research prototype, not a
   medical device, and must not be described as one.
