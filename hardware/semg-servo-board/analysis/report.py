#!/usr/bin/env python3
"""Evaluate the front end and the power tree and write the numbers out.

    python3 hardware/semg-servo-board/analysis/report.py

Produces the JSON that hardware/semg-servo-board/docs/ quotes from and the
figures beside it. Every number in those documents comes from a run of this
script, so a component value changed in afe_model.py or power_model.py reaches
the documentation or fails the check in CI.

None of it is a measurement. No board exists.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from afe_model import (  # noqa: E402
    ELECTRODE_CASES,
    Components,
    bandwidth_headroom,
    common_mode_rejection,
    magnitude,
    mains_rejection,
    noise_budget,
    response_db,
    signal_to_noise_db,
)
from power_model import (  # noqa: E402
    ACTUATORS,
    Battery,
    Board,
    OPERATING_POINTS,
    continuous_current_limit,
    dissipation,
    fuse_recommendation,
    runtime_minutes,
    temperatures,
)

def reproducible(value, digits: int = 9):
    """Round every float in a structure to `digits` significant figures.

    The committed JSON is compared against a fresh run in CI, so it has to come
    out byte for byte the same on a different machine. Full double precision
    does not: these figures are sums and quotients whose last digit depends on
    the order the compiler chose and on whose libm computed the logarithm, and
    a board temperature that reads 75.98463623783785 here and
    75.98463623783783 on a runner fails the check for no reason anybody can
    act on.

    Nine significant figures is far more than any of these numbers deserve,
    given that the most confident of them rests on a datasheet typical.
    """
    if isinstance(value, dict):
        return {key: reproducible(item, digits) for key, item in value.items()}
    if isinstance(value, list):
        return [reproducible(item, digits) for item in value]
    if isinstance(value, bool) or not isinstance(value, float):
        return value
    if value == 0.0 or not math.isfinite(value):
        return value
    exponent = math.floor(math.log10(abs(value)))
    return round(value, digits - 1 - exponent)


DISCLAIMER = (
    "Calculated, not measured. No board has been fabricated, nothing has been "
    "powered and no instrument has been connected. Every figure is arithmetic "
    "from a datasheet parameter and a stated assumption."
)

# Surface EMG amplitude at the skin, for the signal-to-noise figures. The range
# is the usual one quoted for forearm muscles: a light contraction sits at the
# bottom of it and a maximal one at the top.
SIGNAL_CASES = (
    ("light contraction", 20.0),
    ("moderate contraction", 200.0),
    ("strong contraction", 1000.0),
)


def afe_report(components: Components) -> dict:
    decades = [1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 450.0, 500.0, 1000.0, 2000.0]
    response = [
        {
            "hz": hz,
            "gain": magnitude(components, hz),
            "db_relative_to_passband": response_db(components, hz),
        }
        for hz in decades
    ]

    budgets = {}
    for electrode in ELECTRODE_CASES:
        budget = noise_budget(components, electrode)
        budgets[electrode.name] = {
            "electrode_impedance_ohms": electrode.impedance_ohms,
            "noise_bandwidth_hz": budget.noise_bandwidth_hz,
            "total_uv_rms": budget.total_microvolts_rms,
            "dominant_source": budget.dominant.source,
            "terms": [
                {
                    "source": term.source,
                    "uv_rms": term.microvolts_rms,
                    "share_of_power": (term.microvolts_rms ** 2)
                    / max(budget.total_microvolts_rms ** 2, 1e-30),
                    "note": term.note,
                }
                for term in sorted(budget.terms, key=lambda t: -t.microvolts_rms)
            ],
            "signal_to_noise_db": {
                name: signal_to_noise_db(budget, amplitude) for name, amplitude in SIGNAL_CASES
            },
        }

    cmrr = {
        electrode.name: common_mode_rejection(components, electrode)
        for electrode in ELECTRODE_CASES
    }
    worst = min(cmrr.values(), key=lambda entry: entry["combined_db"])

    return {
        "report": "sEMG analogue front end",
        "source": "hardware/semg-servo-board/analysis/afe_model.py",
        "disclaimer": DISCLAIMER,
        "components": {
            "stage1_gain": components.stage1_gain,
            "stage2_gain": components.stage2_gain,
            "total_gain": components.total_gain,
            "highpass_hz": components.highpass_hz,
            "lowpass_hz": components.lowpass_hz,
            "lowpass_q": components.lowpass_q,
            "input_rc_hz": components.input_filter_hz,
            "full_scale_uv": components.full_scale_microvolts,
            "uv_per_count": components.microvolts_per_count,
            "adc_bits": components.adc_bits,
            "adc_noise_lsb_rms": components.adc_noise_lsb_rms,
        },
        "bandwidth": bandwidth_headroom(components),
        "response": response,
        "noise": budgets,
        "common_mode_rejection": cmrr,
        "worst_case_common_mode_rejection": worst,
        "mains": mains_rejection(components, worst["combined_db"]),
    }


def power_report(battery: Battery, board: Board) -> dict:
    points = []
    for point in OPERATING_POINTS:
        heat = dissipation(point.servo_current_a, battery.nominal_v)
        thermal = temperatures(point.servo_current_a, battery.nominal_v, board)
        points.append({
            "name": point.name,
            "note": point.note,
            "servo_current_a": point.servo_current_a,
            "battery_current_a": heat["rails"]["battery_current_a"],
            "dissipation_w": heat["total_w"],
            "dissipation_items_w": heat["items_w"],
            "servo_rail_efficiency": heat["rails"]["servo_rail"]["efficiency"],
            "board_temperature_c": thermal["board_c"],
            "servo_converter_junction_c": thermal["servo_converter_junction_c"],
            "within_thermal_limit": thermal["within_limit"],
            "runtime_minutes": runtime_minutes(point.servo_current_a, battery),
            "shunt_headroom": heat["shunt_headroom"],
        })

    limit = continuous_current_limit(battery.nominal_v, board)
    return {
        "report": "servo drive current and thermal budget",
        "source": "hardware/semg-servo-board/analysis/power_model.py",
        "disclaimer": DISCLAIMER,
        "battery": {
            "cells": battery.cells,
            "nominal_v": battery.nominal_v,
            "full_v": battery.full_v,
            "empty_v": battery.empty_v,
            "capacity_ah": battery.capacity_ah,
        },
        "board": {
            "size_mm": [board.length_mm, board.width_mm],
            "layers": board.layers,
            "theta_ja_c_per_w": board.theta_ja_c_per_w,
            "ambient_c": board.ambient_c,
            "temperature_limit_c": board.temperature_limit_c,
        },
        "actuators": [
            {
                "joint": actuator.joint,
                "stall_current_a": actuator.stall_current_a,
                "rated_torque_nm": actuator.rated_torque_nm,
            }
            for actuator in ACTUATORS
        ],
        "operating_points": points,
        "continuous_current_limit_a": limit,
        "continuous_limit_note": (
            "The largest servo current the board can carry indefinitely inside "
            "the 85 C limit. Above it the arm can still move, in bursts, which "
            "is what a prosthesis actually does."
        ),
        "fuse": fuse_recommendation(),
    }


def write_svg_response(components: Components, path: pathlib.Path) -> None:
    """Bode magnitude of the front end, as an SVG with no dependencies.

    Written by hand rather than with a plotting library so that generating the
    documentation needs nothing but a Python interpreter. It is two hundred
    points and a pair of axes; a dependency for that would be a poor trade.
    """
    width, height = 720, 380
    left, right, top, bottom = 70, 20, 30, 50
    plot_w = width - left - right
    plot_h = height - top - bottom

    points = []
    f_min, f_max = 0.5, 3000.0
    db_min, db_max = -60.0, 5.0
    for i in range(401):
        hz = 10 ** (math.log10(f_min) + (math.log10(f_max) - math.log10(f_min)) * i / 400)
        db = max(db_min, response_db(components, hz))
        x = left + plot_w * (math.log10(hz) - math.log10(f_min)) / (math.log10(f_max) - math.log10(f_min))
        y = top + plot_h * (db_max - db) / (db_max - db_min)
        points.append(f"{x:.2f},{y:.2f}")

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
        f'width="{width}" height="{height}" font-family="sans-serif" font-size="12">',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#fafafa" '
        f'stroke="#333"/>',
    ]

    for decade in (1, 10, 100, 1000):
        x = left + plot_w * (math.log10(decade) - math.log10(f_min)) / (
            math.log10(f_max) - math.log10(f_min))
        lines.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + plot_h}" '
                     f'stroke="#ddd"/>')
        lines.append(f'<text x="{x:.1f}" y="{top + plot_h + 18}" text-anchor="middle">'
                     f'{decade} Hz</text>')
    for db in (0, -10, -20, -30, -40, -50, -60):
        y = top + plot_h * (db_max - db) / (db_max - db_min)
        lines.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" '
                     f'stroke="#ddd"/>')
        lines.append(f'<text x="{left - 8}" y="{y + 4:.1f}" text-anchor="end">{db} dB</text>')

    # The band the design is for.
    x20 = left + plot_w * (math.log10(20.0) - math.log10(f_min)) / (
        math.log10(f_max) - math.log10(f_min))
    x450 = left + plot_w * (math.log10(450.0) - math.log10(f_min)) / (
        math.log10(f_max) - math.log10(f_min))
    lines.append(f'<rect x="{x20:.1f}" y="{top}" width="{x450 - x20:.1f}" height="{plot_h}" '
                 f'fill="#2f6fb0" opacity="0.08"/>')
    lines.append(f'<text x="{(x20 + x450) / 2:.1f}" y="{top + 16}" text-anchor="middle" '
                 f'fill="#2f6fb0">20 to 450 Hz</text>')

    lines.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="#c1440e" '
                 f'stroke-width="2"/>')
    lines.append(f'<text x="{left}" y="{top - 12}">sEMG front end, gain '
                 f'{components.total_gain:.0f}, relative to the passband '
                 f'(calculated, not measured)</text>')
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def write_svg_noise(afe: dict, path: pathlib.Path) -> None:
    """Stacked contribution of each noise source, per electrode case."""
    cases = list(afe["noise"].items())
    width, height = 720, 320
    left, right, top, bottom = 220, 130, 46, 40
    plot_w = width - left - right
    row_h = (height - top - bottom) / max(len(cases), 1)

    worst = max(case["total_uv_rms"] for _, case in cases)
    palette = ["#c1440e", "#2f6fb0", "#4a8a3a", "#8a6d3b", "#6b4c9a", "#3a8a8a", "#999999"]

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
        f'width="{width}" height="{height}" font-family="sans-serif" font-size="11">',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        f'<text x="12" y="20" font-size="13">Input-referred noise, 20 to 450 Hz '
        f'(calculated, not measured)</text>',
    ]

    sources: list[str] = []
    for _, case in cases:
        for term in case["terms"]:
            if term["source"] not in sources:
                sources.append(term["source"])

    for row, (name, case) in enumerate(cases):
        y = top + row * row_h + 6
        lines.append(f'<text x="{left - 10}" y="{y + row_h / 2:.1f}" text-anchor="end">'
                     f'{name}</text>')
        x = left
        for term in sorted(case["terms"], key=lambda t: sources.index(t["source"])):
            share = (term["uv_rms"] ** 2) / max(worst ** 2, 1e-30)
            w = plot_w * share
            if w < 0.4:
                continue
            colour = palette[sources.index(term["source"]) % len(palette)]
            lines.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" '
                         f'height="{row_h - 14:.1f}" fill="{colour}"/>')
            x += w
        lines.append(f'<text x="{x + 8:.1f}" y="{y + row_h / 2:.1f}">'
                     f'{case["total_uv_rms"]:.2f} uV RMS</text>')

    legend_y = height - 16
    x = 12
    for index, source in enumerate(sources):
        colour = palette[index % len(palette)]
        lines.append(f'<rect x="{x}" y="{legend_y - 9}" width="10" height="10" fill="{colour}"/>')
        label = source if len(source) < 34 else source[:31] + "..."
        lines.append(f'<text x="{x + 14}" y="{legend_y}">{label}</text>')
        x += 14 + 7 * len(label) + 12
        if x > width - 120:
            x = 12
            legend_y -= 16
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def summarise(afe: dict, power: dict) -> str:
    lines = []
    lines.append("sEMG front end")
    lines.append(f"  gain                {afe['components']['total_gain']:.1f}")
    lines.append(f"  band                {afe['components']['highpass_hz']:.2f} Hz to "
                 f"{afe['components']['lowpass_hz']:.0f} Hz")
    lines.append(f"  full scale          {afe['components']['full_scale_uv']:.0f} uV at the "
                 f"electrode")
    for name, case in afe["noise"].items():
        lines.append(f"  noise, {name:<34} {case['total_uv_rms']:.2f} uV RMS "
                     f"(dominated by the {case['dominant_source']})")
    worst = afe["worst_case_common_mode_rejection"]
    lines.append(f"  common-mode rejection {worst['combined_db']:.0f} dB, limited by the "
                 f"{worst['binding_limit']} ({worst['electrode']})")
    lines.append(f"  50 Hz at the electrode: {afe['mains']['after_cmrr_uv']:.3f} uV after the "
                 f"amplifier, {afe['mains']['after_digital_notch_uv']:.4f} uV after the notch")
    lines.append("")
    lines.append("Power and thermal")
    for point in power["operating_points"]:
        lines.append(f"  {point['name']:<32} {point['servo_current_a']:5.2f} A servo, "
                     f"{point['dissipation_w']:5.2f} W, board "
                     f"{point['board_temperature_c']:5.1f} C, "
                     f"{point['runtime_minutes']:5.1f} min of battery")
    lines.append(f"  continuous current limit        "
                 f"{power['continuous_current_limit_a']:.2f} A inside "
                 f"{power['board']['temperature_limit_c']:.0f} C")
    lines.append(f"  firmware trips at               {power['fuse']['firmware_trip_a']:.0f} A, "
                 f"fuse at {power['fuse']['fuse_a']:.0f} A")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=pathlib.Path, default=HERE.parent / "analysis")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    components = Components()
    battery = Battery()
    board = Board()

    afe = afe_report(components)
    power = power_report(battery, board)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, payload in (("afe_noise.json", afe), ("power_thermal.json", power)):
        path = args.output_dir / name
        with open(path, "w", encoding="utf-8", newline="\n") as handle:
            json.dump(reproducible(payload), handle, indent=2)
            handle.write("\n")

    write_svg_response(components, args.output_dir / "afe_response.svg")
    write_svg_noise(afe, args.output_dir / "afe_noise.svg")

    if not args.quiet:
        print(summarise(afe, power))
        print()
        print(DISCLAIMER)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
