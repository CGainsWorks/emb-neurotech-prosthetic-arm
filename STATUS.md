# Project status

Last updated: 22 August 2026

**No hardware exists.** No board has been fabricated, no arm has been
manufactured, no firmware has run on silicon and no measurement of any kind
appears anywhere in this repository. Everything below is either software that
runs, a calculation, or a simulation and this document says which for each
item.

## Complete

### Software

- **C++ kinematics library.** Forward kinematics, a closed-form inverse
  solution returning all eight branches, damped least squares as an
  independent cross-check, recursive Newton-Euler dynamics, the mass matrix,
  forward dynamics and synchronised trajectory generation in joint and
  Cartesian space. 73 tests. Checked against the gradient of the potential
  energy, against kinetic energy from link velocities and by round-tripping
  the dynamics, which are three independent routes to the same numbers.
- **Firmware, portable core.** Signal conditioning for eight channels, servo
  limiting, an LSM6DSOX driver over an abstract bus, the COBS and CRC-16 wire
  protocol, the fault supervisor and the rate-monotonic dispatcher. 91 tests.
- **Firmware image.** Cross-compiles for the STM32H743, links, fits in
  24.6 KB of flash and 15.5 KB of RAM and passes a structural check of the
  vector table, the stack pointer, the interrupt entries and the DMA buffer
  placement.
- **ROS 2 control stack.** Five packages: messages, a generated description,
  gesture mapping with confidence gating and dwell, a 100 Hz joint controller
  with gravity feedforward and velocity lead and a board bridge with a
  simulated backend that speaks the real protocol.
- **End-to-end simulation.** Acquisition through to actuation, deterministic,
  in a few seconds, writing JSON that CI checks. 2,800 sEMG frames, 1,400
  telemetry frames, zero decode errors, every uninterrupted move settling.

### Hardware design

- **A1 arm development kit PCB**, revision 1.1. Design archive and PDF in
  `hardware/arm-devkit/`. Imported from the `A1DevKit` repository.
- **sEMG device industrial design**, Shapr3D model and render in
  `hardware/semg-servo-board/`. Imported from the `OpenEMG` repository.
- **sEMG analogue front end.** Three stages with every component value, a
  documented noise budget of 0.92 µV RMS referred to input, a common-mode
  rejection analysis identifying which of three limits binds and a mains
  rejection chain. Calculated by a model in the repository, with tests that
  check the model against physics and against the firmware constants derived
  from it.
- **Servo driver current and thermal budget.** Four operating points, a
  continuous current limit of 6.4 A and a three-level protection hierarchy
  whose ordering is asserted as a test.

### Documentation

- Architecture, the kinematic derivation, the control loop timing budget, the
  safety statement, the clinical limitations statement, the integration
  contract with the classifier, the bring-up procedure and the pinout.
- Every quoted number is checked against the result that produced it by
  `scripts/check_docs.py`, which runs in CI.

## Not started

Everything that needs a physical object.

- **Schematic capture and PCB layout.** The analogue front end and the power
  tree exist as component values and arithmetic. Nothing has been drawn in
  Altium or KiCad, no netlist exists and no Gerbers have been generated.
- **Fabrication and assembly.** No board has been ordered.
- **Bring-up.** [`firmware/docs/BRING_UP.md`](firmware/docs/BRING_UP.md) is
  written and none of its twenty steps has been performed.
- **Measured noise floor.** The 0.92 µV figure is calculated. The procedure
  for measuring it is step 11 of bring-up and it is the single measurement
  this repository most needs.
- **Measured current, temperature and runtime.** All three are arithmetic from
  datasheet parameters.
- **Measured loop timing on the target.** The scheduler collects per-task
  statistics and reports them over the link; nothing has read them, because
  nothing has run.
- **Servo stiffness and lag.** The two numbers the gravity compensation and
  the velocity lead are configured with are design figures for the actuator
  class. Bring-up step 16 measures them.
- **Mechanical design of the arm.** The kinematic and inertial model is a
  design. No parts have been drawn, printed or machined and the URDF
  deliberately uses primitive shapes rather than meshes, because a mesh would
  be a claim about an object that does not exist.
- **Integration with the gesture classifier.** The message contract exists and
  the arm acts on it, driven by a scripted decision stream. The classifier has
  never been connected and
  [`docs/INTEGRATION.md`](docs/INTEGRATION.md) sets out the three reasons it
  cannot simply be wired up: eight channels against sixteen, the aliasing in
  the 200 Hz output stream and the absence of any recorded signal to test
  against.
- **User trials.** None and no ethics approval has been sought, because there
  is nothing to approve.

## Known problems with what does exist

Recorded here rather than left for a reader to find.

- **The 200 Hz stream aliases and cannot not.** Nyquist is 100 Hz and the band
  the classifier reads reaches 99 Hz. The residual is bounded at 12 to 37 dB
  between 100 and 160 Hz and asserted as a test and the clean fix is to
  retrain the classifier at the acquisition rate rather than decimate to meet
  it.
- **The front end clips above 4.9 mV.** A maximal contraction can reach 5 mV
  peak. Raising the gain would make it worse and lowering it would push the
  signal into the converter's noise; 306 is where those two costs are
  comparable.
- **The arm has no position feedback.** The servos have no output-shaft
  encoder, so the telemetry reports what the servo was told rather than where
  it is. Every tracking figure in this repository is knowable only because the
  arm in question is a simulation.
- **The current trip runs on an estimate.** The shunt and its amplifier are on
  the schematic; until a board reads them, the fault layer trips on a
  deliberately pessimistic model of what each channel is drawing.
- **2.2 kg.** Too heavy to wear, before the socket, the battery and the
  electronics.

## Before making this repository public

1. **Confirm the investor and patent position for NeuroTech Biowares Ltd.**
   If a filing is pending or an investor agreement restricts disclosure, this
   repository stays private. This has not been confirmed and is the blocking
   item.
2. No personally recorded sEMG is committed and none should be. Subject
   recordings are personal data. Every sEMG signal here is synthetic and is
   labelled as such; the classifier uses the public Ninapro DB5 dataset and
   lives in `ml-semg-gesture-classifier`.
3. The clinical limitations statement is written and is linked from the first
   screen of the README. It stays there.
