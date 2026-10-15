# Firmware

STM32H743 firmware for the A1 sEMG and servo board: eight-channel acquisition
and conditioning, six servo outputs, an I2C IMU, a fault supervisor and the
link to the host.

> **Never executed.** No board has been fabricated. The image compiles, links,
> fits and is well formed and the portable half passes 91 host-side tests,
> but not one instruction has run on silicon.
> [`docs/BRING_UP.md`](docs/BRING_UP.md) is the procedure for when one exists.

```bash
./run_tests.sh          # host unit tests; needs only a C++ compiler
./build_firmware.sh     # the STM32H743 image; needs arm-none-eabi-gcc
./verify_image.sh       # check the linked image is well formed
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

```
flash   24,584 B of 2 MB
DTCM    15,452 B of 128 KB, leaving 113 KB above the 8 KB stack
```

`verify_image.sh` checks what linking alone does not: the vector table is at
the reset address, the initial stack pointer is inside RAM, the reset vector
is a thumb address pointing at `Reset_Handler`, the three interrupt vectors
point at the handlers that service them and the DMA buffers are in AXI SRAM
rather than in the tightly coupled RAM the DMA controllers cannot reach.

That last one is the single most common way an STM32H7 project loses its
first week and it is visible in the ELF without any hardware at all.

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
