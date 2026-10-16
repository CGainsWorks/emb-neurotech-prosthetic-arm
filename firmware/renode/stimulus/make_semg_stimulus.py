#!/usr/bin/env python3
"""Write the synthetic sEMG stimulus the Renode test replays into the ADC.

    python3 make_semg_stimulus.py OUT.csv [--seconds 3.0] [--seed 20261016]

The output is the voltage at each of the eight ADC pins, sampled at 2 kHz: the
front end's 1.5 V mid-rail bias plus the electrode signal multiplied by the
front end's gain. One header row, then one row per sample, one column per
electrode, in volts.

It is synthetic and contains no recording of anybody. What it contains, per
channel and in electrode terms before the gain:

  * 3 uV RMS of broadband noise throughout, roughly the dry-electrode floor in
    hardware/semg-servo-board/docs/noise_analysis.md;
  * 30 uV of 50 Hz mains pickup throughout, which the firmware's notch exists
    to remove;
  * two contractions, each 400 uV RMS of noise band-limited to 20 to 450 Hz
    with 50 ms raised-cosine edges: electrodes 0 to 3 from 0.8 s to 1.6 s and
    electrodes 4 to 7 from 1.9 s to 2.6 s.

Those are plausible orders of magnitude for surface EMG, chosen so the test has
something unambiguous to look for. They are not a physiological model and no
statement about real signals follows from the firmware's response to them.

Pure Python and seeded, so the file is identical on every machine.
"""

import argparse
import math
import random

RATE_HZ = 2000.0
CHANNELS = 8
MID_RAIL_V = 1.5
# pa/config/board.hpp, kFrontEndGain. The test checks the firmware's own
# constant has not moved away from this.
GAIN = 305.785

NOISE_RMS_V = 3e-6
MAINS_V = 30e-6
MAINS_HZ = 50.0
BURST_RMS_V = 400e-6
EDGE_S = 0.05

# (first channel, last channel, start s, end s)
BURSTS = [(0, 3, 0.8, 1.6), (4, 7, 1.9, 2.6)]


def envelope(t, start, end):
    """Raised-cosine gate: 0 outside, 1 inside, smooth edges."""
    if t <= start or t >= end:
        return 0.0
    if t < start + EDGE_S:
        return 0.5 - 0.5 * math.cos(math.pi * (t - start) / EDGE_S)
    if t > end - EDGE_S:
        return 0.5 - 0.5 * math.cos(math.pi * (end - t) / EDGE_S)
    return 1.0


class BandPass:
    """First-order high-pass at 20 Hz into first-order low-pass at 450 Hz."""

    def __init__(self, low_hz=20.0, high_hz=450.0, rate_hz=RATE_HZ):
        dt = 1.0 / rate_hz
        rc_high = 1.0 / (2.0 * math.pi * low_hz)
        rc_low = 1.0 / (2.0 * math.pi * high_hz)
        self.a_high = rc_high / (rc_high + dt)
        self.a_low = dt / (rc_low + dt)
        self.previous_in = 0.0
        self.high = 0.0
        self.low = 0.0
        # Unit-variance white noise loses power in the band edges; this puts
        # the output back at roughly unit RMS so BURST_RMS_V means what it says.
        self.scale = 1.0 / math.sqrt(2.0 * (high_hz - low_hz) / rate_hz)

    def step(self, x):
        self.high = self.a_high * (self.high + x - self.previous_in)
        self.previous_in = x
        self.low += self.a_low * (self.high - self.low)
        return self.low * self.scale


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("output")
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--seed", type=int, default=20261016)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    filters = [BandPass() for _ in range(CHANNELS)]
    phases = [rng.uniform(0.0, 2.0 * math.pi) for _ in range(CHANNELS)]
    rows = int(round(args.seconds * RATE_HZ))

    with open(args.output, "w", encoding="ascii", newline="\n") as out:
        out.write(",".join(f"ch{i}" for i in range(CHANNELS)) + "\n")
        for n in range(rows):
            t = n / RATE_HZ
            values = []
            for ch in range(CHANNELS):
                electrode = NOISE_RMS_V * rng.gauss(0.0, 1.0)
                electrode += MAINS_V * math.sin(2.0 * math.pi * MAINS_HZ * t + phases[ch])
                gate = 0.0
                for first, last, start, end in BURSTS:
                    if first <= ch <= last:
                        gate = max(gate, envelope(t, start, end))
                burst = filters[ch].step(rng.gauss(0.0, 1.0))
                electrode += gate * BURST_RMS_V * burst
                volts = MID_RAIL_V + GAIN * electrode
                values.append(f"{min(3.0, max(0.0, volts)):.6f}")
            out.write(",".join(values) + "\n")


if __name__ == "__main__":
    main()
