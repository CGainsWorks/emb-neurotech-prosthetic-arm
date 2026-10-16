# Control loop timing budget

From a muscle contracting to the arm moving and what each stage costs.

Three kinds of number appear below and they are not interchangeable. The
document labels every one of them:

| | |
|---|---|
| **measured** | run on a machine, with a clock. All of these are host measurements; nothing has run on the arm's controller, because there is no arm |
| **simulated** | produced by the end-to-end run in [`analysis/sim/control_loop_sim.json`](../analysis/sim/control_loop_sim.json). A property of the design, not of any machine |
| **estimated** | arithmetic from a datasheet. Carries the largest error bars and is flagged wherever it is used |

Regenerate the measured and simulated figures with:

```bash
make sim
./kinematics/build/ik_benchmark --iterations 20000 > analysis/kinematics_latency.json
```

## The budget, end to end

A wearer contracts a muscle. When does the arm start moving and when does it
arrive?

| Stage | Time | Kind | Where it comes from |
|---|---|---|---|
| Window fill | 200 ms | fixed | The classifier's 200 ms window must fill before it can decide |
| Acquisition and conditioning | 0.5 ms | fixed | One 2 kHz sample period; the filters are causal and add group delay inside it |
| Link, board to host | 0.5 ms | estimated | 41 bytes framed at 921600 baud, plus scheduling |
| Classifier inference | 13 ms | estimated | ml-semg-gesture-classifier's arithmetic estimate for an STM32H743 (12.6 ms); 0.22 ms measured on a desktop |
| Wait for the next decision | 0 to 50 ms | fixed | Decisions arrive on a 50 ms stride |
| Dwell | 150 ms | simulated | Three decisions held before the arm acts |
| Plan | 0.00015 ms | measured | `plan_joint_move`, median |
| Control tick | 0 to 10 ms | fixed | The joint loop runs at 100 Hz |
| Control computation | 0.0008 ms | measured | Median; 0.0016 ms at the ninety-ninth percentile |
| Link, host to board | 0.3 ms | estimated | 26 bytes framed at 921600 baud |
| Servo frame | 0 to 3 ms | fixed | The pulse train runs at 333 Hz |
| **Contraction to first movement** | **365 to 428 ms** | | |
| Movement | 1.3 to 1.5 s | simulated | A wrist movement at the planner's scaled limits |
| **Contraction to arrival** | **1.7 to 1.9 s** | | |

The slowest move in the simulated run takes **1679 ms** from the classifier's
first decision to the arm being within a degree of its target and that figure
excludes the 200 ms window and the inference, which belong to the classifier.

## Where the time actually goes

Two stages account for 94% of the delay before the arm moves and neither is
computation.

**The window, at 200 ms.** It is set by the classifier and it is not free to
change: a shorter window costs accuracy and the classifier's own
latency-accuracy study found 200 ms to be the knee.

**The dwell, at 150 ms.** This repository's own choice and the one worth
defending. The classifier is 82% accurate with a 7% false activation rate at
the raw decision level, which is a spurious movement every fifteen seconds.
Acting on every decision would give an arm that twitches. Three decisions at a
50 ms stride absorb an isolated misclassification entirely and the simulated
run exercises it: 28 of its 280 decisions are confident and wrong and the arm
performs 7 movements rather than 35.

The computation is nothing by comparison. Planning a move costs 150
nanoseconds and a control tick costs 800, which together are a ten-thousandth
of the dwell. There is no point optimising either.

## What the control stack costs, measured

From [`analysis/kinematics_latency.json`](../analysis/kinematics_latency.json),
20,000 iterations on an AMD Ryzen under WSL2. Host measurements; the arm's
controller is a different machine and has never been run.

| | Median | p99 |
|---|---|---|
| Forward kinematics | 0.67 µs | 1.15 µs |
| Jacobian | 0.68 µs | 1.05 µs |
| **Inverse kinematics, closed form** | **5.53 µs** | **21.9 µs** |
| Inverse kinematics, damped least squares | 74.4 µs | 10,872 µs |
| Inverse dynamics | 0.96 µs | 2.77 µs |
| Forward dynamics | 5.45 µs | 15.0 µs |
| Plan and sample a trajectory | 0.13 µs | 0.68 µs |

The two inverse solvers are the interesting row. The closed form is thirteen
times quicker at the median and five hundred times quicker at the tail,
because the iterative solver's worst case is exhausting its iteration budget;
it also converges from a neutral seed only 76% of the time, where the closed
form returns every branch every time. That is what the concurrent wrist bought
([`dh_parameters.md`](dh_parameters.md) has the derivation) and it is why the
controller runs the closed form and keeps the numerical one as a cross-check.

## What the loop achieves, simulated

From [`analysis/sim/control_loop_sim.json`](../analysis/sim/control_loop_sim.json).

| | |
|---|---|
| Tracking error, median | 0.011 rad (0.6°) |
| Tracking error, p99 | 0.054 rad (3.1°) |
| Peak torque, shoulder flexion | 6.28 N·m against an 8 N·m rating |
| Torque saturation | never, outside acceleration |
| Decode errors over 5,600 frames | 0 |
| Uninterrupted moves that settled | 3 of 3 |

The tracking error is what the gravity compensation and the velocity lead are
for. Without them a position servo sits wherever its own stiffness balances
the load, which on the shoulder is several degrees and lags while it is
moving by its own derivative-to-proportional ratio, which at a typical reach
speed is several more. Running the same simulation with both disabled
(`sim_demo --no-compensation 0`) is the comparison.

## The board's own loop

The firmware runs five periodic tasks. None has been executed on hardware.
They do run under Renode at the rates below but Renode does not model the
Cortex-M7's pipeline or memory system, so the per-task times the firmware
reports there are not a measurement and are not quoted here. The budget below is what the scheduler is configured for and
`the_default_schedule_is_feasible_on_paper` in
[`firmware/test/test_scheduler.cpp`](../firmware/test/test_scheduler.cpp)
checks that the budget itself adds up.

| Task | Rate | Deadline | Budget | Utilisation |
|---|---|---|---|---|
| Acquire | 2 kHz | 50 µs | 40 µs | 8.0% |
| Servo | 333 Hz | 750 µs | 30 µs | 1.0% |
| Control and safety | 100 Hz | 2.5 ms | 900 µs | 9.0% |
| IMU | 100 Hz | 5 ms | 300 µs | 3.0% |
| Status | 10 Hz | none | 200 µs | 0.2% |
| | | | **total** | **21.2%** |

The Liu and Layland bound for five tasks under rate-monotonic scheduling is
74.3%, so the schedule has room. It needs it: the classifier's inference is
estimated at 13 ms on this part, which is another 130% of a control period if
it ever runs on the board rather than on the host and that is the reason the
current architecture keeps it on the host.

The acquisition deadline is the tight one, at a tenth of its period. A sample
taken late is a sample at the wrong time and the classifier's window assumes
the interval is uniform. Telemetry has no deadline at all, because a late
status message is a late status message.

## Link budget

At 921600 baud with eight data bits, a start bit and a stop bit:

| Stream | Rate | On the wire | Load |
|---|---|---|---|
| sEMG frames | 200 Hz | 47 bytes | 9.4 kB/s |
| Joint telemetry | 100 Hz | 54 bytes | 5.4 kB/s |
| IMU | 100 Hz | 43 bytes | 4.3 kB/s |
| Status | 10 Hz | 29 bytes | 0.3 kB/s |
| Joint commands, inbound | 100 Hz | 32 bytes | 3.2 kB/s |
| | | **total** | **22.6 kB/s, 25% of the line** |

`a_semg_frame_fits_the_link_at_the_rate_it_is_produced` in
[`firmware/test/test_protocol.cpp`](../firmware/test/test_protocol.cpp) asserts
the headroom against the real encoded sizes rather than against these
estimates.

## What would change with a measurement

Every number above that is marked measured comes from a desktop. The ones that
matter and are not measured at all:

- What the five firmware tasks actually cost on an STM32H743. The scheduler
  already collects min, max and mean per task and reports them in the status
  message; the first bring-up session produces them.
- Jitter on the acquisition interrupt, which is what decides whether the
  classifier's window assumption holds.
- The servo's real stiffness and lag, which are the two numbers the gravity
  compensation and the velocity lead are configured with.
  [`firmware/docs/BRING_UP.md`](../firmware/docs/BRING_UP.md) has the
  procedure.
- Round-trip link latency, measured rather than divided out of a baud rate.
