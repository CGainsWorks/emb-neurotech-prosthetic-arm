# sEMG signal conditioning

> **Designed, not built.** No board has been fabricated. Every number here is
> calculated by
> [`analysis/afe_model.py`](../analysis/afe_model.py) and printed by
> [`analysis/report.py`](../analysis/report.py); none of it has been measured,
> because there is nothing to measure it on. See [STATUS.md](../../../STATUS.md).

Eight identical differential channels. One is described here; the others are
copies with different electrode positions.

```
electrode ──┬─ 1 k ──┬────────────┐
            │        │            │  INA333       ┌── 1 uF ── 33 k ──┬── 1 M ──┐
           10 M     1 nF        [+]  G = 10.09    │                  │         │
            │        │            │  ────────────▶│                 [-]  TLV9062
electrode ──┴─ 1 k ──┴────────────┘               │                  │         │
                                 [-]              └──────────────────┴─────────┘
                                                            G = 30.30
                                                                 │
                                            Sallen-Key, 477 Hz, Q 0.74
                                                                 │
                                                    STM32H743 ADC, 16 bit
```

## The three stages and why each is there

### 1. Instrumentation amplifier, gain 10.09

INA333, with an 11 kΩ gain resistor. `G = 1 + 100k / Rg`.

The gain is low on purpose. An electrode half-cell potential is tens of
millivolts of DC and it drifts over minutes; the input stage has to carry it
without saturating. At a gain of 306 in one stage, 30 mV of offset would ask
for 9 V of output from a 3 V rail.

There is a second reason. The INA333 has a 350 kHz gain-bandwidth product, so
at a gain of 10 its bandwidth is 35 kHz, seventy times the top of the signal
band. Putting all the gain here instead would leave 1.1 kHz and the response
would be drooping inside the signal. `test_splitting_the_gain_is_what_keeps_the_bandwidth`
in [`analysis/test_hardware_model.py`](../analysis/test_hardware_model.py)
checks both numbers.

The 1 kΩ series resistors and the 1 nF differential capacitor form a 79 kHz
low pass. That is not for the signal: it is radio-frequency rejection, so that
a phone next to the arm does not rectify in the amplifier's input stage and
appear as a DC shift. The 10 MΩ resistors to mid-rail give the input bias
current somewhere to go; without them the inputs float and the stage walks into
a rail within seconds of the electrodes being applied.

### 2. High pass at 4.8 Hz, then gain 30.30

A 1 µF capacitor into a 33 kΩ resistor, feeding an inverting stage with a 1 MΩ
feedback resistor.

The offset is gone before the signal is made large, which is what makes the
second stage possible at all. 4.8 Hz sits below the lowest motor unit firing
rate worth keeping, around 8 Hz and above the electrode drift.

The firmware runs a second, digital high pass at 5 Hz on top of this. That is
not redundancy: the analogue one protects the second stage from saturating and
the digital one removes what the analogue one leaves, including any offset the
amplifiers themselves contribute.

### 3. Sallen-Key low pass, 477 Hz, Q 0.74

Unequal capacitors (33 nF and 15 nF) with equal 15 kΩ resistors, which gets
close to a Butterworth Q of 0.707 without needing a gain stage to set it.

Surface EMG has essentially no energy above 450 Hz. This is the analogue half
of the anti-alias filter for the 2 kHz converter; the digital half, a
sixth-order Butterworth at 80 Hz, does the work for the 200 Hz output stream.
[`docs/INTEGRATION.md`](../../../docs/INTEGRATION.md) explains why there are two
output rates and what that costs.

## Total gain

| | |
|---|---|
| Stage 1 | 10.09 |
| Stage 2 | 30.30 |
| **Total** | **305.79** |
| Full scale at the electrode | ±4905 µV |
| Resolution | 0.150 µV per count |

Not a round number, because E96 resistors are not round numbers. The firmware
carries the same figure as `kFrontEndGain` in
[`pa/config/board.hpp`](../../../firmware/include/pa/config/board.hpp) and
`test_the_firmware_agrees_with_the_component_values` parses it out of the header
and compares it against the component values here. A resistor changed in one
place and not the other would otherwise produce microvolt readings that are
quietly wrong by a factor.

### Why 306 and not 500

At a gain of 500 the front end clips at 3 mV at the electrode. Surface EMG over
forearm flexors reaches about 5 mV peak during a strong contraction, so a gain
of 500 would clip exactly the contractions the classifier most needs to get
right. 306 puts full scale at 4.9 mV, which still clips the very largest
signals and that is stated in [`noise_analysis.md`](noise_analysis.md) rather
than rounded away.

Going the other way is not free either. The converter's own noise is one of the
two largest terms in the budget and lowering the gain would push the signal
further into it.

## Common-mode rejection and the driven right leg

A person standing near mains wiring carries around a volt of 50 Hz common mode.
Three things attenuate it and they are not interchangeable.

| Limit | With gelled electrodes | With dry electrodes |
|---|---|---|
| Amplifier specification | 100 dB | 100 dB |
| Electrode impedance imbalance | 114 dB | 88 dB |
| Series resistor tolerance (0.1%) | 174 dB | 174 dB |

The resistors are not the problem at any electrode this arm would be used with.
With gel the amplifier's own specification binds; with dry electrodes the
imbalance between the two electrodes overtakes it, which is an argument for gel
rather than for a different amplifier. Two nominally identical electrodes on the
same arm routinely differ by twenty percent and no resistor tolerance helps
with that.

The common-mode input impedance used in that calculation is 1 GΩ, which is the
INA333's 3 pF of input capacitance at 50 Hz rather than its DC figure. At DC the
number would be a hundred times better and the conclusion would be wrong.

The driven right leg is fitted regardless of which limit binds, because it does
something the other two cannot: it reduces the common-mode voltage on the body
rather than rejecting it afterwards. A loop gain of 45 divides the 1 V down to
about 22 mV before the amplifier sees any of it, which matters most when the
interference is large enough to move the inputs outside their range, at which
point rejection is irrelevant.

After the driven right leg, the amplifier's rejection and the firmware's 50 Hz
notch (34 dB, measured through the difference equation in
[`firmware/test/test_dsp.cpp`](../../../firmware/test/test_dsp.cpp)), 1 V of
common mode on the body arrives as about 18 nV at the electrode. That is fifty
times below the 0.92 µV noise floor, which is the margin the design aims for:
the mains term should not be the reason anything is hard to classify.

## Bill of materials, per channel

| Reference | Value | Part | Why |
|---|---|---|---|
| U1 | - | INA333 | 25 nV/√Hz, 10 Hz corner, micropower |
| U2A, U2B | - | TLV9062 | Rail to rail in and out on a 3 V supply |
| R1, R2 | 1 kΩ 0.1% | - | Protection and the RF filter; matched for CMRR |
| R3, R4 | 10 MΩ | - | Bias current return |
| R5 | 11 kΩ 0.1% | - | Sets stage 1 gain |
| R6 | 33 kΩ 1% | - | High pass and stage 2 input |
| R7 | 1 MΩ 1% | - | Stage 2 feedback |
| R8, R9 | 15 kΩ 1% | - | Sallen-Key |
| C1 | 1 nF C0G | - | Differential RF filter |
| C2, C3 | 100 pF C0G | - | Common-mode RF filter |
| C4 | 1 µF X7R | - | High pass |
| C5 | 33 nF C0G | - | Sallen-Key |
| C6 | 15 nF C0G | - | Sallen-Key |

C0G rather than X7R everywhere the capacitor is in the signal path: X7R loses
capacitance with applied voltage and is piezoelectric, so a knock on the board
appears as a signal. In a filter that is a shifted corner frequency; in a
biopotential front end at a gain of 306 it is a movement artefact indistinguishable
from a real one.

## What has not been done

- No schematic capture, no layout, no fabrication, no assembly.
- No SPICE simulation. The transfer function here is derived analytically from
  the component values; a SPICE run would add the amplifiers' own poles and
  their non-ideal behaviour and would be the natural next step before layout.
- No measurement of any kind. Everything above is arithmetic.
