#!/usr/bin/env python3
"""Write the synthetic sEMG recording the end-to-end launch replays.

    python3 make_semg_replay.py OUT.csv [--seed 20261016]

One header row, then one row per 5 ms frame (200 Hz, the board's output
rate): eight raw samples and eight envelopes, in the ADC counts the board's
SemgFrame carries. It is built at compile time by arm_bringup's CMakeLists and
installed next to the launch files.

It is synthetic and contains no recording of anybody. The wearer performs, in
order: rest, wrist flexion (13), rest, wrist pronation (10), rest, a fist (6,
which this mechanism has no fingers for) and rest. Each gesture leans on the
electrodes arm_interface::activation_for() assigns it, which is the same
pattern the simulated board synthesises: the primary electrode is the
gesture id modulo eight, the secondary three along, the rest at 0.15. Muscle
activation follows the intent with a 120 ms time constant, also as the
simulated board does. None of it is a physiological model; it exists so the
classifier interface has something to decide between.
"""

import argparse
import math
import random

CHANNELS = 8
FRAME_S = 0.005
STEP_S = 0.001
TAU_S = 0.12
REST_COUNTS = 60.0
ACTIVE_COUNTS = 2200.0

# (gesture id, seconds held)
TIMELINE = [(0, 1.0), (13, 2.5), (0, 1.5), (10, 2.5), (0, 1.5), (6, 2.0), (0, 1.0)]


def weights_for(gesture):
    """arm_interface::activation_for(), transcribed."""
    if gesture == 0:
        return [0.0] * CHANNELS
    primary = gesture % CHANNELS
    secondary = (primary + 3) % CHANNELS
    return [1.0 if i == primary else 0.65 if i == secondary else 0.15 for i in range(CHANNELS)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("output")
    parser.add_argument("--seed", type=int, default=20261016)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    activation = 0.0
    shape = weights_for(0)
    elapsed = 0.0
    next_frame = 0.0

    with open(args.output, "w", encoding="ascii", newline="\n") as out:
        out.write(",".join([f"s{i}" for i in range(CHANNELS)] +
                           [f"e{i}" for i in range(CHANNELS)]) + "\n")
        for gesture, hold_s in TIMELINE:
            pattern = weights_for(gesture)
            target = 0.0 if gesture == 0 else 1.0
            end = elapsed + hold_s
            # The last gesture's pattern lingers while its activation decays,
            # which is what a relaxing muscle does.
            while elapsed < end - 1e-9:
                activation += (target - activation) * (STEP_S / TAU_S)
                elapsed += STEP_S
                if gesture != 0:
                    shape = pattern
                if elapsed + 1e-9 >= next_frame:
                    next_frame += FRAME_S
                    amplitudes = [REST_COUNTS + activation * shape[i] * (ACTIVE_COUNTS - REST_COUNTS)
                                  for i in range(CHANNELS)]
                    samples = [max(-32767, min(32767, int(a * rng.gauss(0.0, 1.0))))
                               for a in amplitudes]
                    envelopes = [int(round(a)) for a in amplitudes]
                    out.write(",".join(str(v) for v in samples + envelopes) + "\n")


if __name__ == "__main__":
    main()
