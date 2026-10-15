#!/usr/bin/env python3
"""Generate the ROS 2 description from the C++ arm model.

The kinematics library is the single source of truth for the mechanism.
``kinematics/tools/dump_model`` prints it as JSON; this script turns that JSON
into a URDF. Nothing about the geometry is typed in twice and CI regenerates
both files and fails on any difference, so a link length changed in
``kinematics/src/model.cpp`` either reaches the description or stops the build.

The conversion is the only part with any subtlety in it. A Denavit-Hartenberg
row is

    A_i = Rz(theta_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)

with the joint variable inside the product, while a URDF joint applies its
fixed origin first and rotates about its axis afterwards. The two cannot be
matched row for row. What does match is shifting the fixed part one joint
along: URDF link i carries the frame ``F_{i-1} . Rz(theta_i)`` and joint
i + 1 carries ``G_i = Tz(d_i) . Tx(a_i) . Rx(alpha_i)`` as its origin. Then

    C_{i+1} = F_{i-1} . Rz(theta_i) . G_i . Rz(theta_{i+1}) = F_i . Rz(theta_{i+1})

which is the same chain. ``arm_description`` has a test that walks both and
compares them numerically rather than taking this paragraph's word for it.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent


def rotation_x(angle: float) -> list[list[float]]:
    c, s = math.cos(angle), math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def matrix_to_rpy(r: list[list[float]]) -> tuple[float, float, float]:
    """Fixed-axis roll, pitch, yaw, which is what URDF means by rpy.

    The singular case at pitch = +/- 90 degrees is handled rather than ignored:
    the arm's own DH table has alpha = +/- 90 on four of six rows, so it is the
    common case here, not the corner one.
    """
    sy = math.hypot(r[0][0], r[1][0])
    if sy > 1e-9:
        roll = math.atan2(r[2][1], r[2][2])
        pitch = math.atan2(-r[2][0], sy)
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        roll = math.atan2(-r[1][2], r[1][1])
        pitch = math.atan2(-r[2][0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def fixed_transform(d: float, a: float, alpha: float):
    """The part of a DH row that does not depend on the joint angle."""
    return rotation_x(alpha), [a, 0.0, d]


def apply(rotation: list[list[float]], translation: list[float], point: list[float]) -> list[float]:
    return [
        translation[i] + sum(rotation[i][j] * point[j] for j in range(3)) for i in range(3)
    ]


def number(value: float) -> str:
    """Enough digits to round-trip, without the noise of seventeen everywhere."""
    if abs(value) < 1e-15:
        return "0"
    text = f"{value:.12g}"
    return text


def triple(values) -> str:
    return " ".join(number(v) for v in values)


def load_model(model_json: pathlib.Path | None) -> dict:
    if model_json is not None:
        return json.loads(model_json.read_text())

    # Fall back to running the tool, so the script is useful before anything
    # has been committed.
    for candidate in (
        REPO_ROOT / "kinematics" / "build" / "dump_model",
        REPO_ROOT / "kinematics" / "build" / "dump_model.exe",
    ):
        if candidate.exists():
            return json.loads(subprocess.check_output([str(candidate)], text=True))
    raise SystemExit(
        "no model JSON given and kinematics/build/dump_model is not built; "
        "run kinematics/run_tests.sh first or pass --model"
    )


def visual_block(indent: str, length: float, radius: float, axis: str, colour: str) -> str:
    """A cylinder standing in for a link.

    Deliberately primitive. There are no meshes in this repository because
    there is no manufactured arm to have meshes of and a made-up mesh in a
    description is a claim about a physical object that does not exist.
    """
    if length <= 1e-6:
        return ""
    half = length / 2.0
    if axis == "z":
        origin_xyz = [0.0, 0.0, half]
        origin_rpy = [0.0, 0.0, 0.0]
    else:  # x
        origin_xyz = [half, 0.0, 0.0]
        origin_rpy = [0.0, math.pi / 2.0, 0.0]

    lines = []
    for tag in ("visual", "collision"):
        lines.append(f"{indent}<{tag}>")
        lines.append(
            f'{indent}  <origin xyz="{triple(origin_xyz)}" rpy="{triple(origin_rpy)}"/>'
        )
        lines.append(f"{indent}  <geometry>")
        lines.append(
            f'{indent}    <cylinder radius="{number(radius)}" length="{number(length)}"/>'
        )
        lines.append(f"{indent}  </geometry>")
        if tag == "visual":
            lines.append(f'{indent}  <material name="{colour}"/>')
        lines.append(f"{indent}</{tag}>")
    return "\n".join(lines)


def generate(model: dict) -> str:
    chain = model["chain"]
    joints = len(chain)

    out: list[str] = []
    out.append('<?xml version="1.0"?>')
    out.append("<!--")
    out.append("  A1 six-axis prosthetic arm.")
    out.append("")
    out.append("  Generated by scripts/generate_urdf.py from the model in")
    out.append("  kinematics/src/model.cpp. Do not edit: CI regenerates it and fails on")
    out.append("  any difference. Change the model instead.")
    out.append("")
    out.append("  The visual geometry is primitive shapes, not meshes. No arm has been")
    out.append("  manufactured, and a mesh would be a claim about a physical object that")
    out.append("  does not exist.")
    out.append("-->")
    out.append('<robot name="a1_arm">')
    out.append("")
    out.append('  <material name="shell"><color rgba="0.85 0.85 0.87 1.0"/></material>')
    out.append('  <material name="joint"><color rgba="0.25 0.28 0.32 1.0"/></material>')
    out.append('  <material name="tool"><color rgba="0.20 0.45 0.70 1.0"/></material>')
    out.append("")
    out.append('  <link name="base_link">')
    out.append("    <visual>")
    out.append('      <origin xyz="0 0 0.01" rpy="0 0 0"/>')
    out.append('      <geometry><cylinder radius="0.055" length="0.02"/></geometry>')
    out.append('      <material name="joint"/>')
    out.append("    </visual>")
    out.append("    <collision>")
    out.append('      <origin xyz="0 0 0.01" rpy="0 0 0"/>')
    out.append('      <geometry><cylinder radius="0.055" length="0.02"/></geometry>')
    out.append("    </collision>")
    out.append("  </link>")
    out.append("")

    previous_link = "base_link"
    # Joint i carries the fixed part of DH row i - 1; the first joint carries
    # nothing, because there is no row before it.
    pending_rotation = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    pending_translation = [0.0, 0.0, 0.0]

    for index, row in enumerate(chain):
        link_name = f"link_{index + 1}"
        joint_name = row["name"]

        rpy = matrix_to_rpy(pending_rotation)
        out.append(f'  <joint name="{joint_name}" type="revolute">')
        out.append(f'    <parent link="{previous_link}"/>')
        out.append(f'    <child link="{link_name}"/>')
        out.append(
            f'    <origin xyz="{triple(pending_translation)}" rpy="{triple(rpy)}"/>'
        )
        out.append('    <axis xyz="0 0 1"/>')
        out.append(
            f'    <limit lower="{number(row["lower"])}" upper="{number(row["upper"])}"'
            f' effort="{number(row["effort"])}" velocity="{number(row["velocity"])}"/>'
        )
        # Damping and friction are not measured. They are a plausible geartrain
        # loss, present so that a dynamic simulator has something rather than
        # nothing and they are not used by the controller.
        out.append('    <dynamics damping="0.05" friction="0.02"/>')
        out.append("  </joint>")
        out.append("")

        # The next joint carries this row's fixed part.
        rotation, translation = fixed_transform(row["d"], row["a"], row["alpha"])

        # Inertia is given in the DH frame of this link, which sits one fixed
        # transform beyond the URDF link frame.
        com_in_link = apply(rotation, translation, row["com"])
        inertia_rpy = matrix_to_rpy(rotation)
        ixx, iyy, izz, ixy, ixz, iyz = row["inertia"]

        out.append(f'  <link name="{link_name}">')
        out.append("    <inertial>")
        out.append(
            f'      <origin xyz="{triple(com_in_link)}" rpy="{triple(inertia_rpy)}"/>'
        )
        out.append(f'      <mass value="{number(row["mass"])}"/>')
        out.append(
            f'      <inertia ixx="{number(ixx)}" ixy="{number(ixy)}" ixz="{number(ixz)}"'
            f' iyy="{number(iyy)}" iyz="{number(iyz)}" izz="{number(izz)}"/>'
        )
        out.append("    </inertial>")

        # A cylinder from this link's frame towards where the next joint sits,
        # which is the actual mechanical member.
        offset = math.hypot(math.hypot(translation[0], translation[1]), translation[2])
        axis = "x" if abs(translation[0]) > abs(translation[2]) else "z"
        radius = 0.035 if index < 3 else 0.028
        block = visual_block("    ", offset, radius, axis, "shell")
        if block:
            out.append(block)
        else:
            out.append("    <visual>")
            out.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            out.append('      <geometry><sphere radius="0.036"/></geometry>')
            out.append('      <material name="joint"/>')
            out.append("    </visual>")
        out.append("  </link>")
        out.append("")

        previous_link = link_name
        pending_rotation = rotation
        pending_translation = translation

    # The terminal device, rigidly attached beyond the last row's fixed part.
    tool = model["tool"]
    rpy = matrix_to_rpy(pending_rotation)
    out.append('  <joint name="tool_mount" type="fixed">')
    out.append(f'    <parent link="{previous_link}"/>')
    out.append('    <child link="tool_link"/>')
    out.append(f'    <origin xyz="{triple(pending_translation)}" rpy="{triple(rpy)}"/>')
    out.append("  </joint>")
    out.append("")

    ixx, iyy, izz, ixy, ixz, iyz = tool["inertia"]
    out.append('  <link name="tool_link">')
    out.append("    <inertial>")
    out.append(f'      <origin xyz="{triple(tool["com"])}" rpy="0 0 0"/>')
    out.append(f'      <mass value="{number(tool["mass"])}"/>')
    out.append(
        f'      <inertia ixx="{number(ixx)}" ixy="{number(ixy)}" ixz="{number(ixz)}"'
        f' iyy="{number(iyy)}" iyz="{number(iyz)}" izz="{number(izz)}"/>'
    )
    out.append("    </inertial>")
    out.append("    <visual>")
    out.append('      <origin xyz="0 0 0.045" rpy="0 0 0"/>')
    out.append('      <geometry><box size="0.075 0.045 0.09"/></geometry>')
    out.append('      <material name="tool"/>')
    out.append("    </visual>")
    out.append("    <collision>")
    out.append('      <origin xyz="0 0 0.045" rpy="0 0 0"/>')
    out.append('      <geometry><box size="0.075 0.045 0.09"/></geometry>')
    out.append("    </collision>")
    out.append("  </link>")
    out.append("")

    # The frame the controller talks about. Coincident with the last DH frame,
    # which is where the kinematics library puts the tool.
    out.append('  <joint name="tool_tip" type="fixed">')
    out.append('    <parent link="tool_link"/>')
    out.append('    <child link="tool_frame"/>')
    out.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
    out.append("  </joint>")
    out.append('  <link name="tool_frame"/>')
    out.append("")
    out.append("</robot>")

    assert joints == 6, "the description generator assumes a six-axis chain"
    return "\n".join(out) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=pathlib.Path,
        default=REPO_ROOT / "ros2_ws/src/arm_description/config/a1_arm_model.json",
        help="model JSON from kinematics/tools/dump_model",
    )
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=REPO_ROOT / "ros2_ws/src/arm_description/urdf/a1_arm.urdf",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="do not write; exit non-zero if the file on disk is out of date",
    )
    args = parser.parse_args()

    model = load_model(args.model if args.model.exists() else None)
    urdf = generate(model)

    if args.check:
        if not args.output.exists():
            print(f"{args.output} does not exist", file=sys.stderr)
            return 1
        current = args.output.read_text()
        if current != urdf:
            print(
                f"{args.output} is out of date with kinematics/src/model.cpp; "
                "run make model",
                file=sys.stderr,
            )
            return 1
        print(f"{args.output} matches the model")
        return 0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    # newline="\n" explicitly: the default on Windows is CRLF and a
    # description that differs by line endings between platforms fails the
    # regeneration check for no reason anybody can see in a diff.
    with open(args.output, "w", newline="\n", encoding="utf-8") as handle:
        handle.write(urdf)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
