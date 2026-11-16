#!/usr/bin/env python3
"""Regression tests for the arm description.

Every assertion here corresponds to a defect that was actually present in the
Fusion-exported description. They are cheap to run and they fail loudly if a
CAD re-export or a careless edit reintroduces one.
"""

import math
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

MOVABLE_JOINTS = [
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "Slider_39",
]


@pytest.fixture(scope="module")
def robot():
    """Expand the top-level xacro and return the parsed URDF root."""
    share = get_package_share_directory("arm_description")
    xacro_file = f"{share}/urdf/arm.urdf.xacro"
    out = subprocess.run(
        ["xacro", xacro_file, "use_mock_hardware:=true"],
        capture_output=True, text=True, check=True,
    )
    return ET.fromstring(out.stdout)


def _joint(robot, name):
    for j in robot.findall("joint"):
        if j.get("name") == name:
            return j
    raise AssertionError(f"joint {name!r} missing from the URDF")


def _rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def test_all_movable_joints_are_limited(robot):
    """No 'continuous' joints.

    The exporter emits continuous because Fusion has no joint-limit concept.
    A continuous joint tells MoveIt the servo can rotate forever, so it plans
    poses the arm physically cannot reach and the goal simply never arrives.
    """
    for name in MOVABLE_JOINTS:
        j = _joint(robot, name)
        assert j.get("type") in ("revolute", "prismatic"), (
            f"{name} is '{j.get('type')}' - it must be limited"
        )
        limit = j.find("limit")
        assert limit is not None, f"{name} has no <limit>"
        for attr in ("lower", "upper", "effort", "velocity"):
            assert limit.get(attr) is not None, f"{name} <limit> lacks {attr}"
        assert float(limit.get("lower")) < float(limit.get("upper")), (
            f"{name} has lower >= upper"
        )


def test_exactly_one_hardware_component(robot):
    """The export had two <ros2_control> blocks sharing one name.

    The controller_manager rejects duplicate component names, so the whole
    stack fails to start with an error that does not obviously point here.
    """
    blocks = robot.findall("ros2_control")
    assert len(blocks) == 1, f"expected 1 <ros2_control>, found {len(blocks)}"
    names = [b.get("name") for b in blocks]
    assert len(names) == len(set(names))


def test_hardware_covers_every_joint(robot):
    """The generated blocks covered 5 of the 7 joints between them."""
    block = robot.find("ros2_control")
    claimed = {j.get("name") for j in block.findall("joint")}
    assert claimed == set(MOVABLE_JOINTS), (
        f"ros2_control covers {sorted(claimed)}, "
        f"missing {sorted(set(MOVABLE_JOINTS) - claimed)}"
    )


def test_initial_state_is_inside_limits(robot):
    """A mock initial_value outside the joint's limits starts the robot in an
    invalid state, and every plan from it fails.

    The gripper is the live case: its range is entirely negative, so the
    exporter's initial_value of 0.0 was out of bounds.
    """
    block = robot.find("ros2_control")
    for hw_joint in block.findall("joint"):
        name = hw_joint.get("name")
        param = hw_joint.find("state_interface[@name='position']/param")
        if param is None:
            continue
        initial = float(param.text)
        limit = _joint(robot, name).find("limit")
        lo, hi = float(limit.get("lower")), float(limit.get("upper"))
        assert lo <= initial <= hi, (
            f"{name} starts at {initial}, outside its limits [{lo}, {hi}]"
        )


def test_robot_stands_upright(robot):
    """The CAD root frame has the waist axis along +Y, not +Z.

    Loaded as exported the robot lies on its side: RViz's ground plane cuts
    through it and gravity would pull it sideways in any physics sim. The
    world_joint corrects it; this asserts the correction still holds.
    """
    world_joint = _joint(robot, "world_joint")
    assert world_joint.get("type") == "fixed"
    assert world_joint.find("parent").get("link") == "world"
    assert world_joint.find("child").get("link") == "base_link"

    origin = world_joint.find("origin")
    r, p, y = (float(v) for v in origin.get("rpy").split())
    R = _rpy_matrix(r, p, y)

    # joint1's axis, expressed in base_link, then rotated into world.
    j1 = _joint(robot, "joint1")
    j1_o = j1.find("origin")
    jr, jp_, jy = (float(v) for v in j1_o.get("rpy").split())
    Rj = _rpy_matrix(jr, jp_, jy)
    axis_base = [Rj[i][2] for i in range(3)]       # joint1 rotates about its z
    axis_world = [sum(R[i][k] * axis_base[k] for k in range(3)) for i in range(3)]

    assert axis_world[2] > 0.9999, (
        f"waist axis points {axis_world} in world, not straight up. "
        "The base mount rotation is wrong."
    )


def test_base_sits_on_the_ground_plane(robot):
    """The base should rest on z=0, not float or sink."""
    origin = _joint(robot, "world_joint").find("origin")
    z = float(origin.get("xyz").split()[2])
    assert 0.0 < z < 0.5, f"base mount z={z} is not a plausible stand-off"


def test_tool_frame_exists(robot):
    """Planning targets tool0, not a CAD-internal link origin."""
    links = {l.get("name") for l in robot.findall("link")}
    assert "tool0" in links
    assert _joint(robot, "tool0_joint").get("type") == "fixed"
