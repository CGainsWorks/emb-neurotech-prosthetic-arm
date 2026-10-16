# Firmware

STM32H743 firmware for the A1 sEMG and servo board: eight-channel acquisition
and conditioning, six servo outputs, an I2C IMU, a fault supervisor and the
link to the host.

> **Never run on silicon.** No board has been fabricated. The portable half
> passes 91 host-side tests, the image is built with the pinned Arm GNU
> Toolchain and CMake and the same ELF boots and runs under Renode on an
> emulated STM32H743, where 7 Robot Framework tests check what it does. An
> emulator is not a board: see [the limits](#what-the-emulation-does-not-show).
> [`docs/BRING_UP.md`](docs/BRING_UP.md) is the procedure for when one exists.

```bash
./run_tests.sh          # host unit tests; needs only a C++ compiler
export PATH="$(../scripts/install_arm_toolchain.sh):$PATH"   # Arm GNU Toolchain 14.3.rel1
./build_firmware.sh     # the STM32H743 image, with CMake
./verify_image.sh       # check the linked image is well formed
RENODE_DIR="$(../scripts/install_renode.sh)" ./renode/run_tests.sh   # run it on Renode 1.17.0
```

## The split

Everything with judgement in it is in `src/` and is portable C++ with no
register access. Everything that touches a register is in `src/platform/`.

That is the whole structure of the firmware and it is what makes any of it
testable. The portable half compiles into three things: the host test suite,
the cross-compiled image and the ROS 2 bridge, which speaks the link by
running the same encoder the board runs rather than keeping a second
implementation in step with it.

| | |
|---|---|
| `dsp/` | Biquad sections, Butterworth and notch design, a DC block. Transposed direct form II, coefficients computed at start-up rather than tabled |
| `semg/` | One channel and eight of them: DC block, notch, anti-alias, decimate, envelope, rail detection and the RMS estimator the noise floor measurement reads |
| `servo/` | Angle to pulse width, with the stops, the slew limit and the refusals. The last thing before a timer register |
| `imu/` | LSM6DSOX over an abstract I2C bus and a complementary filter |
| `protocol/` | COBS framing, CRC-16 and the six messages. Shared with the host |
| `safety/` | The fault state machine and the only thing allowed to enable the outputs |
| `app/` | The rate-monotonic dispatcher and its timing statistics |
| `config/board.hpp` | Every constant, with the reason attached |

## What the tests actually check

91 tests in seven binaries, in under a second. The ones worth knowing about:

- **Filters measured, not derived.** Sine waves go through the real difference
  equation and the amplitude is measured at the other end, rather than
  evaluating the transfer function the coefficients came from. The second only
  proves the algebra.
- **No single bit flip produces an acceptable frame.** Every bit in a frame is
  flipped in turn and the decoder must reject all of them.
- **A simulated LSM6DSOX**, including the failures that never happen on a
  bench: a device that is not there, a bus that NACKs and a sensor that
  answers but has stopped producing samples. That last one is the dangerous
  case, because a stalled sensor looks exactly like a stationary arm.
- **The protection hierarchy.** An ordinary synchronised move stays under the
  current trip and a jam exceeds it, which is what makes the trip a protection
  rather than decoration.
- **The scheduler across the 32-bit microsecond wrap**, which happens every
  71 minutes and would otherwise freeze the arm an hour in.
- **The anti-alias filter against what the decimator needs**, with the
  residual aliasing bounded rather than ignored.
- **A command that is not a number is refused.** A NaN survives a clamp
  unchanged and would otherwise reach a timer register.

## The image

Built with the Arm GNU Toolchain 14.3.rel1:

```
flash   22,784 B of 2 MB
DTCM    15,428 B of 128 KB, leaving 113 KB above the 8 KB stack
```

`verify_image.sh` checks what linking alone does not: the vector table is at
the reset address, the initial stack pointer is inside RAM, the reset vector
is a thumb address pointing at `Reset_Handler`, the three interrupt vectors
point at the handlers that service them and the DMA buffers are in AXI SRAM
rather than in the tightly coupled RAM the DMA controllers cannot reach.

That last one is the single most common way an STM32H7 project loses its
first week and it is visible in the ELF without any hardware at all.

## Running the image on an emulated STM32H743

[`renode/`](renode) runs the unmodified ELF under Renode 1.17.0, on a
platform description adapted from Renode's own STM32H7 one
([`renode/a1_board.repl`](renode/a1_board.repl)) with what is on this board
attached to it:

| On the board | In the emulation |
|---|---|
| Eight electrodes into ADC1, 2 kHz, DMA | Renode's STM32H7 ADC, each channel replaying one column of a synthetic stimulus file ([`renode/stimulus/make_semg_stimulus.py`](renode/stimulus/make_semg_stimulus.py)) |
| DMA1, circular with half and full interrupts | [`renode/peripherals/STM32H7_DMA.cs`](renode/peripherals/STM32H7_DMA.cs), derived from Renode's STM32DMA, which has no circular mode or half transfer interrupt |
| LSM6DSOX on I2C1 | Renode's STM32 I2C controller and [`renode/peripherals/LSM6DSOX.cs`](renode/peripherals/LSM6DSOX.cs); Renode's LSM6DSO model has no STATUS register, which this driver polls |
| Six servos on TIM1 and TIM3 | Renode's STM32 timers, with every register write logged by a bus hook |
| The host on USART1 | [`renode/peripherals/HostLinkReplay.cs`](renode/peripherals/HostLinkReplay.cs) plays JointCommand frames encoded by [`tools/pa_link.cpp`](tools/pa_link.cpp); the board's output is captured and decoded by the same tool, which is built from this firmware's own protocol code |

[`renode/tests/a1_board.robot`](renode/tests/a1_board.robot) runs three
seconds of virtual time: a host commanding the six joints for 700 ms, two
synthetic contractions on the electrodes and the IMU rolled by ten degrees.
Its seven tests check, all on the emulated board:

- **It boots.** The first boot did not: the reset handler never ran the
  static constructors, so the I2C bus object had no vtable and the first
  call through it jumped to address zero. Fixed in
  [`src/platform/stm32/startup.cpp`](src/platform/stm32/startup.cpp).
- **The link carries every frame.** 1,226 frames in 3 s with no CRC, framing
  or overflow errors: sEMG at 200 Hz, telemetry and IMU at 100 Hz, status at
  10 Hz, with the board's DWT timestamps advancing at 1 MHz of virtual time.
- **The supervisor gates the outputs.** Idle until the first command, active
  while commands arrive, holding when they stop with the command timeout
  fault reported, then idle with the outputs disabled. The timer output
  enables go off, on and off exactly once each and the pins read low after.
- **The servo path puts the right pulse on the timer.** 333 Hz programmed
  against a 200 MHz timer clock, every pulse the telemetry reports written to
  the matching compare register and each joint's pulse at the design
  calibration for its target, within a microsecond.
- **The pins are routed.** The GPIO registers put TIM1 and TIM3 on the servo
  pins, USART1 on PA9 and PA10 and I2C1 on PB6 and PB7.
- **Acquisition follows the stimulus.** Each contraction raises the envelope
  on its own four channels about a hundredfold over rest and leaves the other
  four where they were.
- **The IMU is read over I2C.** Roll from the stimulus accelerometer comes
  back as 10.0 degrees and the die temperature reaches the telemetry.

### What the emulation does not show

- **Timing.** Renode executes instructions, not the Cortex-M7's pipeline,
  caches or flash wait states. The loop times the firmware reports from its
  cycle counter are Renode's, not the part's.
- **The ADC trigger.** Renode's STM32H7 ADC converts at a fixed rate rather
  than on TIM2's trigger. The test checks the firmware programs TIM2 for the
  same 2 kHz but the trigger path itself is not exercised.
- **The waveform on the pins.** With this firmware, which rewrites every
  compare register once a servo frame, the output lines of Renode 1.17.0's
  STM32 timer showed three edges in 10 ms where a 333 Hz waveform has six or
  seven, so the PWM is checked at the registers rather than measured on the
  pins.
- **Anything analogue.** The stimulus is a voltage at the ADC pin. The
  electrodes, the front end and its noise are the models in
  [`../hardware/semg-servo-board`](../hardware/semg-servo-board).
- **Open drain on the I2C pins.** Renode's GPIO model does not implement
  OTYPER.

## Deliberately not here

- **No vendor HAL and no CubeMX.** Nine peripherals are used, written out
  against the reference manual, so every register access is something a reader
  can check.
- **No RTOS.** Five periodic tasks, none of which blocks, all with bounded
  execution. The whole scheduling problem fits in one header and a kernel
  would add a context switch whose jitter nobody here can measure.
- **No dynamic allocation, no exceptions, no RTTI.** Every buffer is a
  fixed-size array sized at compile time.
- **Nothing heavy in an interrupt.** The acquisition handler advances a
  pointer and sets a flag; the filtering happens in the task.

## Two numbers it estimates and should measure

The current trip runs on an estimate from the commanded step, not on the shunt
that is on the schematic, because there is no board to read one from. The
servo's stiffness and lag, which the host-side gravity compensation and
velocity lead are configured with, are design figures for the actuator class.

[`docs/BRING_UP.md`](docs/BRING_UP.md) steps 16 and 18 are where all three
stop being estimates.

## Pin conflict, resolved

TIM1's four channels are on port E rather than the more obvious PA8 to PA11,
because PA9 and PA10 are USART1. [`docs/PINOUT.md`](docs/PINOUT.md) has the
table and the reason.
