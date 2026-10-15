# Safety statement for body-contact electronics

> This is a research prototype, not a medical device. It has never been built,
> powered or worn. [`CLINICAL_LIMITATIONS.md`](CLINICAL_LIMITATIONS.md) says
> what that means and what would have to change.

This document records the hazards the design accounts for and the mechanism
that addresses each one. It is a design record, not a risk management file to
ISO 14971 and it is not a substitute for one.

## The two hazard classes

A myoelectric arm has electronics in contact with skin and actuators with
enough torque to hurt. They fail differently and they are treated separately.

## Electrical: the patient-connected part

| Hazard | What addresses it | State |
|---|---|---|
| Current through the electrodes into the wearer | 1 kΩ in series with every electrode, limiting a rail-to-rail fault to 3 mA | designed |
| Electrostatic discharge through an electrode lead | Clamp diodes at each input, ahead of the series resistor | designed |
| Mains leakage via a connected debugger | **Nothing.** The amplifier ground, the battery and any debug probe share a ground | **not addressed** |
| Battery fault heating the body-worn pack | 15 A fuse at the pack and a pack with its own protection circuit | designed |

The third row is the important one and it is why this must not be worn. There
is no isolation barrier. A debug probe attached to a mains-powered laptop puts
the wearer's skin and mains earth in the same circuit and a single fault in
that laptop's supply becomes a fault across a person.

A device would need the patient-connected part isolated and assessed to
IEC 60601-1 as a type BF applied part. That is a redesign of the power tree
and the link, not an addition to it.

### Why the series resistors are 1 kΩ

They set the worst-case fault current. With the 3 V rail across a single
resistor into a short at the skin, 3 V / 1 kΩ is 3 mA, below the 10 mA at
which a person cannot let go and well below anything that reaches the heart
through intact dry skin at these electrode areas.

They are also what the common-mode rejection calculation treats as the matched
pair, which is why they are 0.1% parts rather than 1%. Both reasons are in
[`signal_conditioning.md`](../hardware/semg-servo-board/docs/signal_conditioning.md).

## Mechanical: the actuators

The arm can produce 8 N·m at the shoulder. At the 0.655 m from the shoulder
axis to the terminal device that is 12 N at the hand, which is enough to
bruise and enough to trap a finger in the mechanism.

| Hazard | What addresses it | Where |
|---|---|---|
| The arm moves at power-on | Outputs stay off until a host sends a command. Power-on is not consent to move | `pa::safety::Supervisor`, state `kIdle` |
| A lost host leaves the arm moving | No command for 30 ms and the arm holds position; no command for 300 ms and it relaxes | `kCommandTimeoutMs` |
| A command outside the mechanism's range | Clamped at the joint limits in the controller and again in the firmware, immediately before the timer register | `clamp_to_limits`, `Servo::update` |
| A step command producing a lunge | Slew limiting per channel, at the joint's rated velocity | `ChannelCalibration::max_rate_rad_s` |
| A corrupt command interpreted as a position | CRC-16 on every frame; a failed frame is counted, not acted on | `pa::protocol::FrameDecoder` |
| A command that is not a number | Refused outright. A NaN survives a clamp unchanged and would reach a timer | `Servo::update`, `state.rejected` |
| A jam burning out an actuator or the board | Current trip at 8 A, thermal trip at 85 °C, both latching | `pa::safety::Supervisor` |
| A firmware fault leaving the pulse train running | Every unrecoverable handler takes the timer outputs down before halting | `startup.cpp`, `stop_outputs()` |
| Filters designed for the wrong sample rate | Checked for stability at start-up; the arm stays in init if any section fails | `FrontEnd::filters_stable` |

Each row has a test. [`firmware/test/test_safety.cpp`](../firmware/test/test_safety.cpp)
covers the supervisor, [`test_servo.cpp`](../firmware/test/test_servo.cpp) the
limiting and the refusals and
[`test_protocol.cpp`](../firmware/test/test_protocol.cpp) the framing, including
that no single bit flip anywhere in a frame produces one the receiver accepts.

### Relaxing rather than holding

When the command timeout expires the arm holds for a grace period and then
stops the pulse train entirely, which makes the servos back-driveable and lets
the arm fall under its own weight.

That is deliberate and it is the less obvious choice. An arm that held its last
position indefinitely with nobody driving it would keep pressing against
whatever it was touching, including a person, with no way to stop it short of
disconnecting the battery. A limp arm can be moved out of the way by hand.

The grace period exists so that one lost packet does not drop a grip.

### What is deliberately not implemented

**No collision detection.** There are no joint torque sensors and no current
sensing per channel, so there is nothing to detect a collision with. The
current trip is a bus-level measurement and will notice a jam, slowly, not a
light contact.

**No mechanical hard stops in the model.** The joint limits are enforced in
software in two places. A built arm would want physical stops as well, because
software limits do not survive a firmware fault and that is a mechanical
design task that has not been done.

**No independent watchdog.** The STM32's internal watchdog is not configured.
A firmware fault that hangs the main loop without hitting a fault handler would
leave the servo timers running from their own clock with the last pulse widths
held. A device would need an external watchdog with a hardware path to the
output enable.

**No dual-channel anything.** Every protection above is single-channel
software running on one core. That is appropriate for a bench prototype and
not for anything attached to a person.

## The link

The host is treated as capable of sending nonsense, because it is: it is a
laptop running a general-purpose operating system and a middleware.

Every command is framed with a CRC, length-checked against the message type it
claims to be, range-checked, rate-limited and timed out. The firmware accepts
a position and never a torque or a pulse width, so the worst a confused host
can ask for is a legal pose at a legal speed.

The wire format is the same implementation on both ends, compiled from
[`firmware/include/pa/protocol`](../firmware/include/pa/protocol), so an
encoder and a decoder cannot drift apart into a state where a well-formed
frame means two different things.

## Before anyone powers one of these

[`firmware/docs/BRING_UP.md`](../firmware/docs/BRING_UP.md) has the order and
the first three steps are: no actuators connected, current-limited bench
supply and the servo rail disconnected until the fault logic has been seen to
work.
