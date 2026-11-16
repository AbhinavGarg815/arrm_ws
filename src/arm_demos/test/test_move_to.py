"""Tests for arm_demos.move_to that need no running robot."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose

from arm_demos.move_to import ARM_JOINTS, NAMED_TARGETS, MoveTo, TOOL_LINK


def _srdf_states():
    path = Path(get_package_share_directory("arm_moveit_config")) / "config" / "arm.srdf"
    root = ET.parse(path).getroot()
    return {
        state.get("name"): {j.get("name"): float(j.get("value")) for j in state.findall("joint")}
        for state in root.findall("group_state") if state.get("group") == "arm"
    }


def test_named_targets_match_the_srdf():
    """NAMED_TARGETS duplicates the SRDF's <group_state>s, because the MoveGroup
    action takes constraints, not names.

    Every time the joint zero moves, 'ready' is re-derived in the SRDF and has
    to be copied here too. It was hand-synced three times in one session; a
    stale copy sends the arm to the old pose with no error at all.
    """
    srdf = _srdf_states()
    assert set(NAMED_TARGETS) == set(srdf), "demo and SRDF define different names"
    for name, values in NAMED_TARGETS.items():
        expected = [srdf[name][j] for j in ARM_JOINTS]
        assert values == pytest.approx(expected, abs=1e-9), f"'{name}' has drifted from the SRDF"


def test_joint_goal_constrains_every_arm_joint():
    goal = MoveTo.joint_goal([0.1, -0.2, 0.3, -0.4, 0.5, -0.6], tolerance=0.02)
    assert [c.joint_name for c in goal.joint_constraints] == ARM_JOINTS
    assert [c.position for c in goal.joint_constraints] == pytest.approx(
        [0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    assert all(c.tolerance_above == c.tolerance_below == 0.02 for c in goal.joint_constraints)


def _pose():
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 0.25, 0.0, 0.25
    pose.orientation.x, pose.orientation.w = 1.0, 0.0      # tool pointing down
    return pose


def test_approach_mode_leaves_spin_about_the_tool_axis_free():
    """'approach' is the default precisely because full orientation goals are
    rarely solvable on this arm. If the free axis quietly became constrained,
    most top-down goals would start failing with NO_IK_SOLUTION."""
    goal = MoveTo.pose_goal(_pose(), orientation_tol=0.1, orientation_mode="approach")
    (oc,) = goal.orientation_constraints
    assert oc.link_name == TOOL_LINK
    assert oc.absolute_x_axis_tolerance == pytest.approx(0.1)
    assert oc.absolute_y_axis_tolerance == pytest.approx(0.1)
    assert oc.absolute_z_axis_tolerance == pytest.approx(2.0 * math.pi)


def test_full_mode_pins_all_three_axes():
    (oc,) = MoveTo.pose_goal(_pose(), orientation_tol=0.1,
                             orientation_mode="full").orientation_constraints
    assert (oc.absolute_x_axis_tolerance, oc.absolute_y_axis_tolerance,
            oc.absolute_z_axis_tolerance) == pytest.approx((0.1, 0.1, 0.1))


def test_none_mode_is_position_only():
    goal = MoveTo.pose_goal(_pose(), position_tol=0.01, orientation_mode="none")
    assert goal.orientation_constraints == []
    (pc,) = goal.position_constraints
    assert pc.link_name == TOOL_LINK
    assert pc.constraint_region.primitives[0].dimensions[0] == pytest.approx(0.01)
    assert pc.constraint_region.primitive_poses[0].position.z == pytest.approx(0.25)
