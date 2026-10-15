# Bring-up procedure

> **Not yet performed.** No board has been fabricated and nothing in this
> document has been done. It is written now, while the reasoning behind each
> step is fresh, rather than reconstructed later from memory.

Every step says what to expect and what it means if the expectation is wrong.
A step whose expected value is "unknown" is one that produces a number this
repository currently has to estimate and those are marked.

## Before power

1. **Visual and continuity.** Shorts between the 6 V servo rail, the 3.3 V
   logic rail and ground. Check the 3.0 V analogue rail against both.
2. **No actuators connected.** The servo connectors stay empty until step 14.
3. **Bench supply, not a battery.** 11.1 V with the current limit set to
   150 mA. A short that the fuse would not notice will be obvious here.

A LiPo pack will deliver tens of amps into a fault without complaining. The
whole point of the first power-up is to find the fault with something that
will not.

## Power

4. **Logic rail.** Apply 11.1 V. Expect under 30 mA with the microcontroller
   held in reset and 3.30 V ±2% at the logic rail.
   *More than 50 mA means a short; 0 V means the buck is not switching, which
   is usually the enable pin or the bootstrap capacitor.*
5. **Analogue rail.** Expect 3.00 V ±1% from the LDO and under 1 mV of ripple
   at the buck's 400 kHz switching frequency. That last one matters more than
   it looks: at a gain of 306, 1 mV on the reference is 3 µV at the electrode,
   which is three times the entire noise budget.
6. **Servo rail, unloaded.** Expect 6.00 V ±3%.
7. **Raise the current limit** to 1 A once all three rails are correct.

## Processor

8. **SWD connects.** Attach a debug probe and read the device ID.
   *Nothing here is isolated. A probe attached to a mains-powered laptop puts
   anything touching the electrodes and mains earth in the same circuit, so
   nobody is wearing anything at any point in this document.*
9. **Flash the image and confirm the clock.** `pa_firmware.bin`, built by
   `./firmware/build_firmware.sh`. Toggle a GPIO in the main loop and check
   the system clock on a scope: 400 MHz through the prescalers gives a 200 MHz
   AHB and a timer configured for 1 ms should produce exactly that.
   *If `configure_clocks()` returns false the part is still on its 64 MHz
   internal oscillator and every rate in the firmware is off by a factor of
   six. The status message reports it as `kFilterUnstable`, because a filter
   designed for 2 kHz and run at 333 Hz is not a filter.*
10. **Status messages arrive.** Open the link at 921600 baud and confirm a
    status frame every 100 ms with a CRC that checks.

## Acquisition

11. **Noise floor, inputs shorted.** Short each channel's inputs together and
    to the bias rail. Read `Channel::rms()` over the link and convert with
    `counts_to_microvolts`.
    **Expected: 0.92 µV RMS.** Predicted, not measured; the budget is in
    [`noise_analysis.md`](../../hardware/semg-servo-board/docs/noise_analysis.md).
    *Within a factor of two is a good first board. A factor of five is the
    layout, not the design: check the analogue ground, the LDO and whether
    the buck's switch node runs under the input traces.*
    **This is the measurement the repository most needs.**
12. **Frequency response.** Inject a 1 mV sine at the electrode terminals
    through a 1000:1 divider and sweep 1 Hz to 1 kHz.
    **Expected:** flat between 20 and 450 Hz, −3 dB at 4.8 Hz and 477 Hz,
    −76 dB at 50 Hz with the notch enabled. The predicted curve is
    [`afe_response.svg`](../../hardware/semg-servo-board/analysis/afe_response.svg).
13. **Mains rejection, on a person.** Electrodes on a forearm, arm still,
    mains equipment nearby.
    **Expected: under 1 µV of 50 Hz at the electrode** after the driven right
    leg, the amplifier and the digital notch.
    *This is the first step involving a person and it needs the isolation
    question answered first. See
    [`CLINICAL_LIMITATIONS.md`](../../docs/CLINICAL_LIMITATIONS.md): as the
    design stands, the answer is that it should not be done.*

## Actuation

14. **One servo, no load.** Connect channel 1 only, with the arm
    disassembled. Command the centre position and confirm a 1500 µs pulse on a
    scope before confirming the horn moves.
15. **The fault logic, before the arm is assembled.** With one servo
    connected:
    - Stop sending commands. Expect the arm to hold for 30 ms, then relax
      after 300 ms, with `kCommandTimeout` set in the status message.
    - Send a command with a deliberately wrong CRC. Expect the frame counted
      as a link error and the position unchanged.
    - Stall the horn by hand. Expect the current estimate to rise and
      `kOverCurrent` to latch at 8 A and expect it to stay latched until
      cleared.
    *Every one of these has a host-side test in
    [`test_safety.cpp`](../test/test_safety.cpp). This step is checking that
    the same logic behaves the same way with a real actuator on the end of it,
    which is a different question.*
16. **Servo stiffness and lag.** The two numbers the controller is configured
    with and the two it currently estimates.
    - **Stiffness.** Command a position, hang a known mass at a known radius
      and measure the deflection. Repeat per joint. The result is
      `compliance_rad_per_nm` in
      [`control.yaml`](../../ros2_ws/src/arm_control/config/control.yaml).
      *Currently 0.012 rad/N·m, which is a design figure for the actuator
      class and has never been checked.*
    - **Lag.** Command a constant-velocity ramp and measure the steady
      following error. Divide by the velocity. The result is
      `velocity_lead_s`.
      *Currently 0.064 s, from the same design figure.*
    Until both are measured, set them to zero. The arm will droop and lag
    visibly, which is correct behaviour for an uncompensated position servo
    and is better than a compensation pointing the wrong way.
17. **All six, assembled, supported.** Hold the arm so it cannot fall.
    Command the resting pose. Expect under 10° of error anywhere.
18. **Current and thermal.** Run the simulated demo sequence against the real
    board. Log the shunt current and the board temperature.
    **Expected: under 2.5 A and under 55 °C** for the "typical reach" case in
    [`thermal_budget.md`](../../hardware/semg-servo-board/docs/thermal_budget.md).
    *Replace the estimator in `pa::servo::estimate_current` with the shunt
    reading once this agrees and tighten the trip if the measurement allows.*

## Loop timing

19. **Read the scheduler's own statistics.** The status message carries the
    control task's min, max and mean. The scheduler collects all five tasks.
    **Expected:** acquisition under 40 µs, control under 900 µs, no overruns.
    Those are the budget in
    [`CONTROL_LOOP_TIMING.md`](../../docs/CONTROL_LOOP_TIMING.md) and they
    are a budget, not a prediction.
20. **Acquisition jitter.** Toggle a GPIO in the acquisition task and measure
    the interval on a scope over a minute.
    **Expected: 500 µs with under 5 µs of jitter.** This is the one that
    matters most for the classifier, whose window assumes a uniform interval.

## After all of it

Three numbers in this repository stop being estimates and become
measurements and the documents that quote them should be updated in the same
commit as the data:

| Number | Currently | Becomes |
|---|---|---|
| Noise floor | 0.92 µV, calculated | measured, with electrodes shorted and on skin |
| Bus current | estimated from the commanded step | the shunt |
| Loop timing | a budget | the scheduler's own statistics |

Nothing above has been done and [STATUS.md](../../STATUS.md) says so.
