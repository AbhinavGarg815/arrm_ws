#!/usr/bin/env python3
"""Consistency between the MoveIt configuration and the robot description.

Several values here are necessarily written down twice, because MoveIt and
ros2_control each want their own file format. Duplicated data drifts. These
tests make the drift a build failure instead of a behaviour you discover on the
arm.
"""

import math
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

DESCRIPTION = Path(get_package_share_directory("arm_description"))
MOVEIT = Path(get_package_share_directory("arm_moveit_config"))


def _yaml(path):
    return yaml.safe_load(path.read_text())


@pytest.fixture(scope="module")
def parameters():
    return _yaml(DESCRIPTION / "config" / "joint_parameters.yaml")["joints"]


@pytest.fixture(scope="module")
def srdf():
    return ET.parse(MOVEIT / "config" / "arm.srdf").getroot()


def test_moveit_limits_mirror_joint_parameters(parameters):
    """joint_limits.yaml must equal joint_parameters.yaml, the source of truth.

    MoveIt's time parameterisation reads joint_limits.yaml. If it drifts above
    the real limits, trajectories are timed faster than the firmware allows and
    the Mega's watchdog relaxes the servo mid-move.
    """
    moveit = _yaml(MOVEIT / "config" / "joint_limits.yaml")["joint_limits"]
    assert set(moveit) == set(parameters), "the two files name different joints"
    for joint, source in parameters.items():
        mirror = moveit[joint]
        assert mirror["has_velocity_limits"] and mirror["has_acceleration_limits"]
        assert mirror["max_velocity"] == pytest.approx(source["velocity"], abs=1e-6), joint
        assert mirror["max_acceleration"] == pytest.approx(source["acceleration"], abs=1e-6), joint


def test_named_states_are_inside_the_joint_limits(parameters, srdf):
    """A named state outside the limits cannot be planned to, and the error
    blames the goal rather than the SRDF."""
    for state in srdf.findall("group_state"):
        for joint in state.findall("joint"):
            name, value = joint.get("name"), float(joint.get("value"))
            lo, hi = parameters[name]["lower"], parameters[name]["upper"]
            assert lo <= value <= hi, (
                f"group_state '{state.get('name')}': {name}={value} outside [{lo}, {hi}]")


def test_moveit_controllers_match_ros2_controllers():
    """Every controller MoveIt hands trajectories to must exist in ros2_control
    and claim the same joints.

    A mismatch is the most common reason a plan succeeds in RViz and then
    nothing moves: move_group finds no controller for the joints and reports
    'Unable to identify any set of controllers'.
    """
    moveit = _yaml(MOVEIT / "config" / "moveit_controllers.yaml")["moveit_simple_controller_manager"]
    ros2 = _yaml(DESCRIPTION / "config" / "ros2_controllers.yaml")
    declared = ros2["controller_manager"]["ros__parameters"]

    for name in moveit["controller_names"]:
        assert name in declared, f"MoveIt uses '{name}', ros2_controllers.yaml does not declare it"
        assert name in ros2, f"'{name}' is declared but has no parameter block"
        assert moveit[name]["joints"] == ros2[name]["ros__parameters"]["joints"], (
            f"'{name}' claims different joints in the two files")


def test_collision_matrix_names_real_links(srdf):
    """A typo in a <disable_collisions> link name is silently ignored by MoveIt,
    which leaves that pair checked (safe) but the matrix no longer means what
    it says."""
    urdf = subprocess.run(
        ["xacro", str(DESCRIPTION / "urdf" / "arm.urdf.xacro"), "use_mock_hardware:=true"],
        capture_output=True, text=True, check=True).stdout
    urdf_links = {link.get("name") for link in ET.fromstring(urdf).findall("link")}
    for entry in srdf.findall("disable_collisions"):
        for attr in ("link1", "link2"):
            assert entry.get(attr) in urdf_links, f"unknown link '{entry.get(attr)}'"

# Hard boundaries each planning limit must stay clear of - whichever of the
# firmware soft limit, a measured mechanical stop or the encoder seam comes
# first for that joint, in degrees:
#
#   joint1  arm3.cpp "base" row          -120.0 / +120.0
#   joint2  arm3.cpp "shoulder" row       -90.0 /  +90.0  (arm structure)
#   joint3  arm3.cpp "elbow" row         -119.5 / +126.5  (stops -119.7/+126.6)
#   joint5  measured stops                -50.5 /  +70.8
#   joint6  encoder seam, count 0/4096   -180.0 / +180.0
#
# joint4 is absent deliberately: its boundary is the wrist harness, which is
# unknown and cannot be measured by rotating the joint.
HARD_BOUNDARY_DEG = {
    "joint1": (-120.0, 120.0),
    "joint2": (-90.0, 90.0),
    "joint3": (-119.5, 126.5),
    "joint5": (-50.5, 70.8),
    "joint6": (-180.0, 180.0),
}

#: Clearance demanded on top of the joint's goal tolerance. See the margin rule
#: at the top of joint_parameters.yaml.
MARGIN_CLEARANCE_DEG = 3.0


def test_limits_keep_the_margin_rule(parameters):
    """Every planning limit sits >= goal tolerance + 3 deg inside its hard boundary.

    The goal tolerance must be in the sum because the JointTrajectoryController
    declares a goal reached anywhere inside it, so a goal placed ON a limit may
    settle that far beyond it. Without the margin the arm can therefore drive
    itself into a mechanical stop (a stalled servo is what browned out the wrist,
    PROJECT.md 4.44) or across the encoder seam (which loses the joint outright,
    4.19).

    This exists because the first attempt at these margins was out by a tenth of
    a degree on joint3 and nothing would have caught it.
    """
    tol = _yaml(DESCRIPTION / "config" / "ros2_controllers.yaml")
    goal_tol = tol["arm_controller"]["ros__parameters"]["constraints"]
    failures = []
    for joint, (hard_lo, hard_hi) in HARD_BOUNDARY_DEG.items():
        lo = math.degrees(parameters[joint]["lower"])
        hi = math.degrees(parameters[joint]["upper"])
        needed = math.degrees(goal_tol[joint]["goal"]) + MARGIN_CLEARANCE_DEG
        for edge, margin in (("lower", lo - hard_lo), ("upper", hard_hi - hi)):
            if margin < needed - 1e-9:
                failures.append(
                    f"{joint} {edge}: {margin:.2f} deg of margin, needs {needed:.2f} "
                    f"(goal tolerance {math.degrees(goal_tol[joint]['goal']):.2f} "
                    f"+ {MARGIN_CLEARANCE_DEG:.1f} clearance)")
    assert not failures, "margin rule violated:\n  " + "\n  ".join(failures)


def test_margin_rule_covers_every_hard_limited_joint(parameters):
    """Catch a joint added to the robot and forgotten here.

    joint4 (harness, unmeasurable) and the gripper are the only exemptions, and
    both are named so that a NEW joint cannot slip through as a third one.
    """
    exempt = {"joint4", "Slider_39"}
    missing = set(parameters) - set(HARD_BOUNDARY_DEG) - exempt
    assert not missing, (
        f"these joints have no hard boundary recorded, so the margin rule is "
        f"not being checked for them: {sorted(missing)}")


def test_report_clamp_fits_inside_the_margins(parameters):
    """arm_hardware's reported_position() clamp must stay inside the margins.

    The driver pulls a measured position back inside its limits when it is less
    than kReportClamp outside them. That is only safe while the margin is larger
    than the clamp: otherwise a clamped reading could hide a joint that is
    actually at a stop or over the seam. The two numbers live in different
    languages, so nothing but a test ties them together.
    """
    header = (Path(__file__).parents[2] / "arm_hardware" / "include" / "arm_hardware"
              / "arm_system_hardware.hpp").read_text()
    match = re.search(r"kReportClamp\s*=\s*([0-9.]+)", header)
    assert match, "could not find kReportClamp in arm_system_hardware.hpp"
    clamp_deg = math.degrees(float(match.group(1)))

    for joint, (hard_lo, hard_hi) in HARD_BOUNDARY_DEG.items():
        lo = math.degrees(parameters[joint]["lower"])
        hi = math.degrees(parameters[joint]["upper"])
        for edge, margin in (("lower", lo - hard_lo), ("upper", hard_hi - hi)):
            assert margin > clamp_deg, (
                f"{joint} {edge}: margin {margin:.2f} deg is not larger than the "
                f"{clamp_deg:.2f} deg reported_position() clamp, so a clamped reading "
                f"could hide a real excursion past the hard boundary")
