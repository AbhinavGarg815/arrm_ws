#!/usr/bin/env python3
"""Plan and execute a motion, from the command line.

    # joint-space goal, by name (names come from the SRDF)
    ros2 run arm_demos move_to --named ready
    ros2 run arm_demos move_to --named home

    # joint-space goal, explicit (radians, 6 values)
    ros2 run arm_demos move_to --joints 0.2 -0.6 1.9 1.4 -0.7 0

    # Cartesian goal: tool0 at x y z, approaching from straight above, free to
    # spin about its own axis (the default --orientation approach)
    ros2 run arm_demos move_to --pose 0.25 0.0 0.25

    # the same, pinning all three axes, or position only
    ros2 run arm_demos move_to --pose 0.25 0.0 0.25 --orientation full
    ros2 run arm_demos move_to --pose 0.25 0.0 0.25 --orientation none

    # plan only, do not move
    ros2 run arm_demos move_to --named ready --plan-only

This drives the /move_action MoveGroup action directly: moveit_py is not
packaged for Jazzy, and MoveGroupInterface would mean a compiled package for a
one-shot command. More verbose, but every field of the request is visible, so a
rejected plan can be read rather than guessed at.
"""

import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             OrientationConstraint, PlanningOptions,
                             PositionConstraint)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
PLANNING_GROUP = "arm"
TOOL_LINK = "tool0"
BASE_FRAME = "world"

# Joint-space named targets. These duplicate the SRDF's <group_state> entries
# because the MoveGroup action takes explicit constraints, not names — the
# resolution normally done inside MoveGroupInterface. Keep them in step with
# arm_moveit_config/config/arm.srdf.
NAMED_TARGETS = {
    "home":  [0.0,  0.0, 0.0, 0.0,  0.0, 0.0],
    "ready": [0.0001, -0.5910, 1.8856, 1.3775, -0.6572, -0.0015],
}

# MoveIt reports these as negative integers in MoveItErrorCodes; the names are
# far more useful than the numbers when something fails at 2am.
ERROR_NAMES = {
    1: "SUCCESS",
    -1: "FAILURE",
    -2: "PLANNING_FAILED",
    -3: "INVALID_MOTION_PLAN",
    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -5: "CONTROL_FAILED",
    -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -7: "TIMED_OUT",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -31: "NO_IK_SOLUTION",
}


class MoveTo(Node):

    def __init__(self):
        super().__init__("arm_move_to")
        self._client = ActionClient(self, MoveGroup, "move_action")
        self._joint_state = None
        self.create_subscription(JointState, "joint_states",
                                 self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def wait_for_stack(self, timeout_sec=30.0):
        """Block until move_group is up AND joint states are flowing.

        Both matter. Planning against a robot whose state is unknown is the
        classic cause of "START_STATE_IN_COLLISION" on a perfectly fine robot:
        MoveIt falls back to all-zeros, which for this arm's gripper is outside
        its own joint limits.
        """
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(
                "move_group is not running. Start it with:\n"
                "  ros2 launch arm_moveit_config demo.launch.py")
            return False

        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while self._joint_state is None:
            if self.get_clock().now().nanoseconds > deadline:
                self.get_logger().error(
                    "No /joint_states. Is joint_state_broadcaster active?\n"
                    "  ros2 control list_controllers")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    # ------------------------------------------------------------- goals ---
    @staticmethod
    def joint_goal(values, tolerance=0.01):
        c = Constraints(name="joint_goal")
        for name, value in zip(ARM_JOINTS, values):
            c.joint_constraints.append(JointConstraint(
                joint_name=name,
                position=float(value),
                tolerance_above=tolerance,
                tolerance_below=tolerance,
                weight=1.0,
            ))
        return c

    @staticmethod
    def pose_goal(pose: Pose, position_tol=0.01, orientation_tol=0.1,
                  orientation_mode="approach"):
        """Position, plus as much orientation as you actually need.

        position_tol is a sphere radius in metres. 10 mm is a deliberate floor:
        the measured error budget is ~3.2 mm of following error plus 3.6 mm of
        backlash at a 0.4 m reach, so tighter asks for accuracy the mechanism
        does not have.

        orientation_mode decides how much of the tool's orientation is pinned:

          "full"      all three axes fixed. Six constraints on six joints with
                      joint4 and joint5 limited to +/-90 deg, so most positions
                      have NO solution at a given full orientation.
          "approach"  (default) the tool's +Z is pinned, rotation about it free -
                      what a gripper needs, and it reaches far more poses.
                      Implemented by opening the tolerance about the tool's own
                      Z axis to 2*pi while keeping X and Y tight.
          "none"      position only.
        """
        c = Constraints(name="pose_goal")

        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[position_tol])
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = TOOL_LINK
        pc.constraint_region.primitives.append(region)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        if orientation_mode != "none":
            oc = OrientationConstraint()
            oc.header.frame_id = BASE_FRAME
            oc.link_name = TOOL_LINK
            oc.orientation = pose.orientation
            oc.absolute_x_axis_tolerance = orientation_tol
            oc.absolute_y_axis_tolerance = orientation_tol
            oc.absolute_z_axis_tolerance = (
                orientation_tol if orientation_mode == "full" else 2.0 * math.pi)
            oc.weight = 1.0
            c.orientation_constraints.append(oc)

        return c

    # ------------------------------------------------------------- execute -
    def go(self, constraints, plan_only=False, velocity_scale=0.3,
           acceleration_scale=0.3, planning_time=5.0, attempts=10):
        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        request.goal_constraints = [constraints]
        request.num_planning_attempts = attempts
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = velocity_scale
        request.max_acceleration_scaling_factor = acceleration_scale
        # Empty start state means "use the current state from the planning scene
        # monitor", which is what we want: it reads the real joint states rather
        # than whatever this process last believed.
        request.start_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions(
            plan_only=plan_only,
            look_around=False,
            replan=False,
        )
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(
            f"requesting {'plan' if plan_only else 'plan + execute'} "
            f"(vel {velocity_scale:.0%}, acc {acceleration_scale:.0%})")

        send = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("move_group rejected the goal")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        outcome = result_future.result()

        code = outcome.result.error_code.val
        name = ERROR_NAMES.get(code, f"UNKNOWN({code})")

        if outcome.status != GoalStatus.STATUS_SUCCEEDED or code != 1:
            self.get_logger().error(f"failed: {name}")
            if code == -31:
                self.get_logger().error(
                    "  No IK solution. The wrist on this arm is offset by "
                    "11.7 mm, so poses near the workspace edge often have no "
                    "solution even when the position looks reachable.")
            elif code == -12:
                self.get_logger().error(
                    "  Goal is in collision. Check it in RViz: the collision "
                    "geometry is convex hulls of the real parts, so it is "
                    "bigger than the visual meshes suggest at the joints.")
            return False

        planned = outcome.result.planned_trajectory.joint_trajectory
        if planned.points:
            duration = planned.points[-1].time_from_start
            seconds = duration.sec + duration.nanosec * 1e-9
            self.get_logger().info(
                f"{name}: {len(planned.points)} waypoints, {seconds:.2f} s")
        else:
            self.get_logger().info(f"{name}: already at the goal")
        return True


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="move_to", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--named", choices=sorted(NAMED_TARGETS),
                        help="a named joint configuration from the SRDF")
    target.add_argument("--joints", nargs=6, type=float, metavar="RAD",
                        help="six joint angles in radians")
    target.add_argument("--pose", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help=f"{TOOL_LINK} position in the {BASE_FRAME} frame, metres")
    parser.add_argument("--rpy", nargs=3, type=float, metavar=("R", "P", "Y"),
                        default=None,
                        help="tool orientation in radians; default points straight down")
    parser.add_argument("--plan-only", action="store_true",
                        help="plan and report, but do not move the arm")
    parser.add_argument("--velocity", type=float, default=0.3,
                        help="velocity scaling, 0-1 (default 0.3)")
    parser.add_argument("--position-tolerance", type=float, default=0.01,
                        help="Cartesian goal tolerance in metres (default 0.01)")
    parser.add_argument("--orientation", choices=["approach", "full", "none"],
                        default="approach",
                        help=("how much of the tool orientation to pin. "
                              "'approach' (default) fixes the approach direction "
                              "but leaves spin about it free - this is what a "
                              "gripper wants and it reaches far more of the "
                              "workspace. 'full' pins all three axes. "
                              "'none' is position only."))
    parser.add_argument("--orientation-tolerance", type=float, default=0.1,
                        help="orientation tolerance in radians (default 0.1 ~ 5.7 deg)")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = MoveTo()
    try:
        if not node.wait_for_stack():
            return 1

        if args.named:
            constraints = node.joint_goal(NAMED_TARGETS[args.named])
            node.get_logger().info(f"target: named pose '{args.named}'")
        elif args.joints:
            constraints = node.joint_goal(args.joints)
            node.get_logger().info(f"target: joints {args.joints}")
        else:
            roll, pitch, yaw = args.rpy if args.rpy else (math.pi, 0.0, 0.0)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = args.pose
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            pose.orientation.w = cr * cp * cy + sr * sp * sy
            pose.orientation.x = sr * cp * cy - cr * sp * sy
            pose.orientation.y = cr * sp * cy + sr * cp * sy
            pose.orientation.z = cr * cp * sy - sr * sp * cy
            constraints = node.pose_goal(
                pose, position_tol=args.position_tolerance,
                orientation_tol=args.orientation_tolerance,
                orientation_mode=args.orientation)
            described = {"full": f"rpy ({roll:.2f}, {pitch:.2f}, {yaw:.2f})",
                         "approach": f"approach along rpy ({roll:.2f}, {pitch:.2f}, _), spin free",
                         "none": "any orientation"}[args.orientation]
            node.get_logger().info(
                f"target: {TOOL_LINK} at {tuple(args.pose)}, {described}")

        ok = node.go(constraints, plan_only=args.plan_only,
                     velocity_scale=args.velocity,
                     acceleration_scale=args.velocity)
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
