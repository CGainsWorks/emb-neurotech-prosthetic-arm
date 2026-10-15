#!/usr/bin/env python3
"""The simulated run still behaves the way the design says it should.

    python3 scripts/check_sim_result.py analysis/sim/control_loop_sim.json

Only the fields that are properties of the design are checked. The host
timings in the same file are a property of whatever machine produced them and
are expected to differ between a laptop and a CI runner; asserting on them
would make the build fail for reasons that have nothing to do with the code.

What is checked is what a reader would use the file to conclude: that the
pipeline carried every frame it should have, that the gating absorbed the
misclassifications rather than acting on them, that every move the arm was
asked to make finished and that no actuator was asked to hold more than it is
rated for.
"""

from __future__ import annotations

import json
import pathlib
import sys


def check(report: dict) -> list[str]:
    problems: list[str] = []

    def fail(message: str) -> None:
        problems.append(message)

    duration = report["duration_s"]
    frames = report["frames"]
    classifier = report["classifier"]
    simulated = report["simulated_time"]
    actuation = report["actuation"]

    # --- the pipeline carried everything -----------------------------------
    # 200 Hz, 100 Hz and 100 Hz, over the run, give or take the frame that
    # straddles the end.
    for name, rate in (("semg", 200.0), ("telemetry", 100.0), ("imu", 100.0)):
        expected = duration * rate
        if abs(frames[name] - expected) > max(4.0, 0.01 * expected):
            fail(f"{frames[name]} {name} frames, expected about {expected:.0f}")

    if frames["decode_errors"] != 0:
        fail(f"{frames['decode_errors']} frames failed to decode; the wire format disagrees "
             "with itself")

    # --- the gating did its job --------------------------------------------
    if classifier["deliberate_misclassifications"] < 10:
        fail("the run did not contain enough wrong decisions to be testing the gating")
    # Seven scripted gestures. Accepting many more than that means the dwell let
    # the deliberate misclassifications through, which is the failure the whole
    # mechanism exists to prevent.
    if classifier["gestures_accepted"] > 10:
        fail(f"{classifier['gestures_accepted']} gestures accepted from "
             f"{classifier['decisions']} decisions, which means wrong ones got through")
    if classifier["gestures_accepted"] < 5:
        fail(f"only {classifier['gestures_accepted']} gestures accepted; the arm barely moved")

    # --- the arm got where it was sent -------------------------------------
    settles = simulated["settle_from_decision_ms"]
    if len(settles) < 3:
        fail(f"only {len(settles)} move(s) settled; the arm is not following its trajectories")
    if settles:
        # From the classifier's decision to the arm being within a degree of
        # the target. docs/CONTROL_LOOP_TIMING.md adds the classifier's own
        # window and inference to this.
        if simulated["max_settle_ms"] > 4000.0:
            fail(f"slowest move took {simulated['max_settle_ms']:.0f} ms from decision to rest")

    if simulated["p99_tracking_error_rad"] > 0.09:
        fail(f"p99 tracking error {simulated['p99_tracking_error_rad']:.3f} rad is more than "
             "five degrees")

    # --- the actuators were not over-asked ---------------------------------
    for index, (peak, rated) in enumerate(
            zip(actuation["peak_torque_nm"], actuation["rated_torque_nm"])):
        if peak > rated + 1e-9:
            fail(f"joint {index + 1} peaked at {peak:.2f} N m against a {rated:.1f} N m rating")

    return problems


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2

    path = pathlib.Path(sys.argv[1])
    if not path.exists():
        print(f"{path} does not exist; run the simulation first", file=sys.stderr)
        return 1

    report = json.loads(path.read_text(encoding="utf-8"))
    problems = check(report)

    if problems:
        print(f"{path} does not describe a run that behaved:\n", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1

    print(f"{path}: the simulated run behaved as the design says it should.")
    print(f"  {report['classifier']['gestures_accepted']} gestures accepted from "
          f"{report['classifier']['decisions']} decisions, "
          f"{report['classifier']['deliberate_misclassifications']} of them deliberately wrong")
    print(f"  {report['frames']['semg']} sEMG, {report['frames']['telemetry']} telemetry and "
          f"{report['frames']['imu']} IMU frames, {report['frames']['decode_errors']} decode "
          "errors")
    print(f"  slowest move {report['simulated_time']['max_settle_ms']:.0f} ms from decision to "
          "rest")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
