"""The URDF and the C++ model describe the same arm.

The description is generated from ``kinematics/src/model.cpp`` by
``scripts/generate_urdf.py`` and the conversion from Denavit-Hartenberg rows
to URDF joints shifts each row's fixed part one joint along. That is the kind
of derivation that is either right or subtly wrong and reading it does not
settle which.

So this walks both chains numerically. Forward kinematics is computed twice
over a few thousand random configurations, once from the DH table and once
from the URDF joint origins and the two poses are compared. A sign error in
an alpha, a transform applied to the wrong link, or a link length that reached
one file and not the other all show up here as millimetres.
"""

from __future__ import annotations

import json
import math
import pathlib
import random
import xml.etree.ElementTree as ElementTree

import pytest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent
MODEL_JSON = PACKAGE_ROOT / "config" / "a1_arm_model.json"
URDF = PACKAGE_ROOT / "urdf" / "a1_arm.urdf"


# --- the smallest amount of linear algebra that will do ---------------------


def identity():
    return ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0])


def multiply(a, b):
    ar, at = a
    br, bt = b
    rotation = [
        [sum(ar[i][k] * br[k][j] for k in range(3)) for j in range(3)] for i in range(3)
    ]
    translation = [at[i] + sum(ar[i][k] * bt[k] for k in range(3)) for i in range(3)]
    return rotation, translation


def rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return ([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], [0.0, 0.0, 0.0])


def rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return ([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], [0.0, 0.0, 0.0])


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return ([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0])


def translate(x, y, z):
    return ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [x, y, z])


def rpy_to_matrix(roll, pitch, yaw):
    # URDF rpy is fixed-axis, applied x then y then z, which composes as
    # Rz . Ry . Rx.
    return multiply(rot_z(yaw), multiply(rot_y(pitch), rot_x(roll)))


def dh_transform(theta, d, a, alpha):
    return multiply(
        rot_z(theta), multiply(translate(0.0, 0.0, d), multiply(translate(a, 0.0, 0.0), rot_x(alpha)))
    )


# --- fixtures ---------------------------------------------------------------


@pytest.fixture(scope="module")
def model():
    assert MODEL_JSON.exists(), f"{MODEL_JSON} is missing; run make model"
    return json.loads(MODEL_JSON.read_text())


@pytest.fixture(scope="module")
def urdf():
    assert URDF.exists(), f"{URDF} is missing; run make model"
    return ElementTree.parse(URDF).getroot()


def joint_elements(urdf):
    return {joint.get("name"): joint for joint in urdf.findall("joint")}


def origin_of(element):
    origin = element.find("origin")
    if origin is None:
        return identity()
    xyz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (origin.get("rpy") or "0 0 0").split()]
    rotation, _ = rpy_to_matrix(*rpy)
    return rotation, xyz


# --- the tests --------------------------------------------------------------


def test_the_urdf_parses_and_names_every_joint(model, urdf):
    joints = joint_elements(urdf)
    for row in model["chain"]:
        assert row["name"] in joints, f"{row['name']} is not in the description"
        assert joints[row["name"]].get("type") == "revolute"
    assert "tool_mount" in joints
    assert joints["tool_mount"].get("type") == "fixed"


def test_the_chain_is_connected_and_has_no_loops(urdf):
    links = {link.get("name") for link in urdf.findall("link")}
    children = []
    parents = []
    for joint in urdf.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        assert parent in links, f"{parent} is used but not declared"
        assert child in links, f"{child} is used but not declared"
        parents.append(parent)
        children.append(child)

    # Exactly one link is nobody's child: the root.
    roots = links - set(children)
    assert roots == {"base_link"}
    # And no link is the child of two joints, which would make it a graph
    # rather than a tree.
    assert len(children) == len(set(children))


def test_joint_limits_match_the_model(model, urdf):
    joints = joint_elements(urdf)
    for row in model["chain"]:
        limit = joints[row["name"]].find("limit")
        assert float(limit.get("lower")) == pytest.approx(row["lower"], abs=1e-9)
        assert float(limit.get("upper")) == pytest.approx(row["upper"], abs=1e-9)
        assert float(limit.get("effort")) == pytest.approx(row["effort"], abs=1e-9)
        assert float(limit.get("velocity")) == pytest.approx(row["velocity"], abs=1e-9)


def test_the_total_mass_matches_the_model(model, urdf):
    total = sum(
        float(link.find("inertial/mass").get("value"))
        for link in urdf.findall("link")
        if link.find("inertial/mass") is not None
    )
    assert total == pytest.approx(model["total_mass_kg"], abs=1e-9)


def test_every_inertia_is_physically_possible(urdf):
    for link in urdf.findall("link"):
        inertia = link.find("inertial/inertia")
        if inertia is None:
            continue
        ixx = float(inertia.get("ixx"))
        iyy = float(inertia.get("iyy"))
        izz = float(inertia.get("izz"))
        assert ixx > 0 and iyy > 0 and izz > 0
        # The triangle inequality on the principal moments. A tensor that
        # breaks it corresponds to no distribution of mass and a dynamic
        # simulator will happily run on it and produce nonsense.
        assert ixx + iyy >= izz - 1e-12
        assert iyy + izz >= ixx - 1e-12
        assert izz + ixx >= iyy - 1e-12


def test_the_urdf_chain_reproduces_the_dh_forward_kinematics(model, urdf):
    """The one that matters.

    Walks the URDF the way robot_state_publisher does, walks the DH table the
    way the C++ library does and compares the tool pose. Anything wrong in the
    conversion shows up here.
    """
    chain = model["chain"]
    joints = joint_elements(urdf)
    random.seed(20260822)

    worst_position = 0.0
    worst_rotation = 0.0

    for _ in range(3000):
        q = [random.uniform(row["lower"], row["upper"]) for row in chain]

        # Denavit-Hartenberg, distal convention, as the library computes it.
        dh_pose = identity()
        for angle, row in zip(q, chain):
            dh_pose = multiply(
                dh_pose, dh_transform(angle + row["theta_offset"], row["d"], row["a"], row["alpha"])
            )

        # The URDF, as a kinematic tree: fixed origin, then rotate about the
        # joint axis, for each joint in turn, then the fixed tool mount.
        urdf_pose = identity()
        for angle, row in zip(q, chain):
            joint = joints[row["name"]]
            axis = [float(v) for v in (joint.find("axis").get("xyz")).split()]
            assert axis == [0.0, 0.0, 1.0], "the walk below assumes a z axis"
            urdf_pose = multiply(urdf_pose, origin_of(joint))
            urdf_pose = multiply(urdf_pose, rot_z(angle))
        urdf_pose = multiply(urdf_pose, origin_of(joints["tool_mount"]))

        position = math.dist(dh_pose[1], urdf_pose[1])
        rotation = max(
            abs(dh_pose[0][i][j] - urdf_pose[0][i][j]) for i in range(3) for j in range(3)
        )
        worst_position = max(worst_position, position)
        worst_rotation = max(worst_rotation, rotation)

    assert worst_position < 1e-9, f"tool position differs by {worst_position:.3e} m"
    assert worst_rotation < 1e-9, f"tool orientation differs by {worst_rotation:.3e}"


def test_the_home_pose_is_where_the_model_says(model, urdf):
    """A pose that can be checked by hand, so the test above cannot be
    self-consistently wrong."""
    chain = model["chain"]
    joints = joint_elements(urdf)

    pose = identity()
    for row in chain:
        pose = multiply(pose, origin_of(joints[row["name"]]))
    pose = multiply(pose, origin_of(joints["tool_mount"]))

    d1 = chain[0]["d"]
    a2 = chain[1]["a"]
    d4 = chain[3]["d"]
    d6 = chain[5]["d"]

    # A nanometre. The URDF stores the joint origins as text at twelve
    # significant figures, so the chain cannot reproduce the model exactly and
    # is not expected to; a nanometre over a 0.76 m arm is fifteen orders of
    # magnitude below anything mechanical.
    assert pose[1][0] == pytest.approx(a2, abs=1e-9)
    assert pose[1][1] == pytest.approx(0.0, abs=1e-9)
    assert pose[1][2] == pytest.approx(d1 - d4 - d6, abs=1e-9)


def test_the_description_declares_a_tool_frame(urdf):
    links = {link.get("name") for link in urdf.findall("link")}
    assert "tool_frame" in links
    assert "tool_link" in links
