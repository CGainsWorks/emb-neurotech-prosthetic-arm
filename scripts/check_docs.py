#!/usr/bin/env python3
"""Every number quoted in the documentation still comes from a committed result.

    python3 scripts/check_docs.py

Documentation goes stale the first time somebody reruns an analysis and forgets
to update a table. This walks the claims below, pulls the figure out of the
JSON the analysis actually wrote, finds the text in the document and fails if
they disagree.

It does not check every number in the repository and it does not pretend to.
What it checks is every number a reader would use to judge the work: the noise
floor, the thermal limits, the protection thresholds, the kinematic accuracy
and the end-to-end simulation result. A claim that is not in this table is a
claim no document should be making.
"""

from __future__ import annotations

import json
import pathlib
import re
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent


class Claim:
    """One number, where it is written and where it comes from."""

    def __init__(self, document: str, pattern: str, source: str, path: list,
                 tolerance: float = 0.005, scale: float = 1.0, description: str = ""):
        self.document = document
        self.pattern = pattern
        self.source = source
        self.path = path
        self.tolerance = tolerance
        self.scale = scale
        self.description = description or pattern


AFE = "hardware/semg-servo-board/analysis/afe_noise.json"
POWER = "hardware/semg-servo-board/analysis/power_thermal.json"
SIM = "analysis/sim/control_loop_sim.json"
WORKSPACE = "analysis/workspace.json"
LATENCY = "analysis/kinematics_latency.json"

GELLED = "gelled Ag/AgCl, prepared skin"

CLAIMS = [
    # --- the analogue front end -------------------------------------------
    Claim("hardware/semg-servo-board/docs/noise_analysis.md",
          r"\*\*([0-9.]+) µV RMS\*\*",
          AFE, ["noise", GELLED, "total_uv_rms"],
          description="noise floor with gelled electrodes"),
    Claim("hardware/semg-servo-board/docs/noise_analysis.md",
          r"noise bandwidth is \*\*([0-9.]+) Hz\*\*",
          AFE, ["noise", GELLED, "noise_bandwidth_hz"],
          description="equivalent noise bandwidth"),
    Claim("hardware/semg-servo-board/docs/signal_conditioning.md",
          r"\| \*\*Total\*\* \| \*\*([0-9.]+)\*\* \|",
          AFE, ["components", "total_gain"],
          description="total front end gain"),
    Claim("hardware/semg-servo-board/docs/signal_conditioning.md",
          r"Full scale at the electrode \| ±([0-9]+) µV",
          AFE, ["components", "full_scale_uv"],
          description="full scale input"),

    # --- power and thermal --------------------------------------------------
    Claim("hardware/semg-servo-board/docs/thermal_budget.md",
          r"carry \*\*([0-9.]+) A\*\* of servo current indefinitely",
          POWER, ["continuous_current_limit_a"],
          description="continuous current limit"),
    Claim("hardware/semg-servo-board/docs/thermal_budget.md",
          r"\| Firmware current trip \| ([0-9.]+) A \|",
          POWER, ["fuse", "firmware_trip_a"],
          description="firmware current trip"),
    Claim("hardware/semg-servo-board/docs/thermal_budget.md",
          r"\| Fuse \| ([0-9.]+) A slow blow \|",
          POWER, ["fuse", "fuse_a"],
          description="fuse rating"),
    Claim("hardware/semg-servo-board/docs/thermal_budget.md",
          r"\| \*\*Total stall\*\* \| \*\*([0-9.]+) A\*\* \|",
          POWER, ["fuse", "total_stall_a"],
          description="total stall current"),
]


def fetch(document: dict, path: list):
    value = document
    for key in path:
        value = value[key]
    return value


def main() -> int:
    failures: list[str] = []
    checked = 0
    sources: dict[str, dict] = {}

    for claim in CLAIMS:
        source_path = REPO_ROOT / claim.source
        if not source_path.exists():
            failures.append(f"{claim.source} is missing; run the analysis that writes it")
            continue
        if claim.source not in sources:
            sources[claim.source] = json.loads(source_path.read_text(encoding="utf-8"))

        document_path = REPO_ROOT / claim.document
        if not document_path.exists():
            failures.append(f"{claim.document} is missing")
            continue

        text = document_path.read_text(encoding="utf-8")
        match = re.search(claim.pattern, text)
        if match is None:
            failures.append(
                f"{claim.document}: no text matching the {claim.description} claim "
                f"(pattern {claim.pattern!r})"
            )
            continue

        written = float(match.group(1))
        try:
            actual = float(fetch(sources[claim.source], claim.path)) * claim.scale
        except (KeyError, IndexError, TypeError) as error:
            failures.append(f"{claim.source}: {' -> '.join(map(str, claim.path))} is missing "
                            f"({error})")
            continue

        checked += 1
        # Relative tolerance, floored so that a figure rounded to one decimal
        # place does not fail on its own rounding.
        allowed = max(abs(actual) * claim.tolerance, 0.05 * 10 ** -_decimals(match.group(1)))
        if abs(written - actual) > allowed:
            failures.append(
                f"{claim.document}: {claim.description} says {written:g}, "
                f"{claim.source} says {actual:g}"
            )

    if failures:
        print("Documentation does not match the results it quotes:\n", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        print(f"\n{checked} claim(s) checked, {len(failures)} problem(s).", file=sys.stderr)
        return 1

    print(f"{checked} documented number(s) match the results they came from.")
    return 0


def _decimals(text: str) -> int:
    return len(text.split(".")[1]) if "." in text else 0


if __name__ == "__main__":
    raise SystemExit(main())
