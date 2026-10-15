# Servo drive: current and thermal budget

> **Calculated, not measured.** No board exists, nothing has been powered and
> no temperature has been observed. Every figure comes from
> [`analysis/power_model.py`](../analysis/power_model.py) and is arithmetic from
> a datasheet parameter and a stated assumption.

Regenerate with:

```bash
python3 hardware/semg-servo-board/analysis/report.py
```

## The power tree

```
3S LiPo, 11.1 V nominal, 2.2 Ah
  │
  ├── 15 A slow-blow fuse ── 5 mR shunt ── INA181A2 ──▶ current sense
  │
  ├── TPS568230 synchronous buck ──▶ 6.0 V, servo rail, 8 A rated
  │                                    └── six servo channels
  │
  └── LMR33620 synchronous buck ──▶ 3.3 V, logic rail
                                     ├── STM32H743, 200 mA
                                     └── TPS7A4700 LDO ──▶ 3.0 V analogue, 12 mA
```

The analogue rail comes from an LDO rather than straight from the buck. It costs
9 mW and it removes the buck's switching ripple from a 3 V reference that sits
under a 306-times amplifier: at that gain, 1 mV of ripple on the reference is
3 µV at the electrode, which is three times the entire noise budget.

## The actuators

| Joint | Stall current | Rated torque | Worst-case gravity torque |
|---|---|---|---|
| Shoulder rotation | 2.5 A | 8.0 N·m | 0 N·m (vertical axis) |
| Shoulder flexion | 2.5 A | 8.0 N·m | 5.0 N·m |
| Elbow flexion | 2.0 A | 6.0 N·m | 1.8 N·m |
| Forearm rotation | 1.2 A | 2.5 N·m | 0.7 N·m |
| Wrist flexion | 1.2 A | 2.5 N·m | 0.5 N·m |
| Tool roll | 0.8 A | 1.2 N·m | 0 N·m (symmetric about its axis) |
| **Total stall** | **10.2 A** | | |

The gravity figures come from
[`kinematics/test/test_dynamics.cpp`](../../../kinematics/test/test_dynamics.cpp),
which samples 200,000 configurations across the whole joint space and asserts
that no configuration inside the mechanical stops asks an actuator for more
continuous torque than it is rated for. The ratings in this table and the ones
in the kinematic model are the same numbers and
`test_every_actuator_can_hold_the_load_the_dynamics_predicts` checks that they
have not drifted apart.

## Operating points

| | Servo current | Battery current | Dissipation | Board temperature | Runtime |
|---|---|---|---|---|---|
| All six holding, unloaded | 0.61 A | 0.41 A | 0.75 W | 47.5 °C | 4 h 25 m |
| A typical reach | 2.40 A | 1.40 A | 0.97 W | 54.1 °C | 1 h 16 m |
| Every joint moving at once | 5.50 A | 3.14 A | 1.70 W | 76.0 °C | 34 m |
| **Every joint stalled (a fault)** | 10.20 A | 5.75 A | 3.64 W | **134 °C** | 18 m |

Ambient 25 °C, thermal resistance 30 °C/W for the whole assembly to still air.
That figure is the pessimistic end of the 25 to 40 °C/W range a four-layer board
of this size with two-ounce inner planes would fall in; the alternative to
over-estimating it is a thermal trip in service.

Runtime assumes 80% of the pack is usable before the cells reach their floor.

## Continuous current

The board can carry **6.4 A** of servo current indefinitely without exceeding
85 °C. That is above the 5.5 A a synchronised move draws and below the 10.2 A a
jam draws, which is the interesting part: an ordinary movement is fine forever
and a fault is not fine for long.

A prosthesis moves in bursts and idles between them, so a limit that forbade
anything above the continuous figure would make the arm useless. The design
allows the bursts and catches the case where they stop being bursts.

## Three protections, at three speeds

| | Threshold | What it catches | Recoverable |
|---|---|---|---|
| Firmware current trip | 8 A | A jam, inside one control period | Yes, by service call |
| Firmware thermal trip | 85 °C board | Sustained current above 6.4 A | Yes, by service call |
| Fuse | 15 A slow blow | A short in the wiring | No |

The ordering is deliberate and it is the part worth checking. The current trip
sits above the 5.5 A an ordinary movement draws, so a movement never trips it
and below the 10.2 A a jam draws, so a jam always does. The fuse sits above
everything the arm can legitimately draw, because a fuse that opens drops a limb
and cannot be reset by software.

`test_the_protection_hierarchy_is_ordered` in
[`analysis/test_hardware_model.py`](../analysis/test_hardware_model.py) asserts
all four inequalities and
`test_the_firmware_trips_where_the_budget_says_it_should` parses the two
thresholds out of
[`pa/config/board.hpp`](../../../firmware/include/pa/config/board.hpp) and
compares them with this document's source.

## Where the watts go, at 5.5 A

| | Dissipation |
|---|---|
| Microcontroller | 0.66 W |
| Servo rail converter (in package) | 0.44 W |
| Servo rail inductor | 0.30 W |
| Current shunt | 0.15 W |
| Logic rail converter and inductor | 0.10 W |
| Analogue front end | 0.04 W |
| **Total** | **1.70 W** |

The microcontroller is the largest single term at every operating point below a
jam, which is worth knowing before anybody spends effort on converter
efficiency. It is also the one that could be reduced without touching the
analogue design: the classifier's own latency model says the arm meets its
deadline at 400 MHz with 55% of a control period to spare and clocking down to
200 MHz would halve this term.

The servo rail converter runs at 91% efficiency at 5.5 A and its junction
sits 17 °C above the board.

## The current estimate the firmware trips on

The firmware trips on an *estimate*, not a measurement, because the shunt
reading is not available on a board that does not exist. The estimator is in
[`pa/servo/driver.cpp`](../../../firmware/src/servo/driver.cpp): quiescent draw
is a fixed fraction of stall and the rest scales with how far each channel is
from where it has been told to be, saturating at stall.

It is deliberately pessimistic. The alternative to over-estimating is a fuse
that opens or a regulator that folds back and both of those drop a limb.

When there is a board, the INA181A2 on the 5 mΩ shunt replaces it: 12 A through
5 mΩ at a gain of 50 is 3.0 V, which is full scale on the converter, so the
sense chain resolves 0.18 mA a count and needs no adjustment. Until then the
estimator is what the trip runs on and the telemetry says so, reporting it in
`JointTelemetry.bus_current_a` with the message definition noting that it is
estimated.

## What has not been done

- No schematic, no layout, no fabrication.
- No measurement of current, efficiency, temperature or runtime.
- No transient analysis. The buck's response to a step from 0.6 A to 5.5 A when
  six servos start together is not modelled here and it is what decides the
  output capacitor. That is a pre-layout task.
- No inrush analysis. A 2.2 Ah pack into the input capacitance needs a soft
  start or a precharge resistor and neither is designed.
