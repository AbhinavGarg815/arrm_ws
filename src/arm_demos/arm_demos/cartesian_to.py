#!/usr/bin/env python3
"""Move the tool in a STRAIGHT LINE to a point, instead of by any path that works.

    # straight down 8 cm from wherever the tool is now
    ros2 run arm_demos cartesian_to --relative 0 0 -0.08

    # straight line to an absolute point, keeping the current tool orientation
    ros2 run arm_demos cartesian_to --pose 0.30 0.05 0.25

    # same, but also rotate the tool to a given orientation along the way
    ros2 run arm_demos cartesian_to --pose 0.30 0.05 0.25 --rpy 0 1.5708 0

    # see what fraction is reachable without moving anything
    ros2 run arm_demos cartesian_to --relative 0 0 -0.10 --plan-only

Use this instead of move_to when the SHAPE of the path matters - approaching or
retracting from something, where the tool must come straight down onto a part
rather than arrive by whatever arc OMPL found. It interpolates in Cartesian
space and solves IK at each step.

A straight line is a stronger demand than "any path", and it fails in ways
joint-space planning does not: the line can leave the reachable workspace
part-way along, pass through a singularity, or be blocked with no way around.
So the service reports a FRACTION of the line it solved, and this refuses to
execute below --min-fraction rather than quietly stopping short.
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

PLANNING_GROUP = "arm"
TOOL_LINK = "tool0"
BASE_FRAME = "world"


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class CartesianMove(Node):
    def __init__(self):
        super().__init__("arm_cartesian_to")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan_srv = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def tool_pose(self, timeout=5.0):
        """Where the tool is now, from TF.

        Needed even when only a target POINT is given, because a Cartesian path
        needs a full target pose - and the sensible default for the orientation
        is "whatever it is now", i.e. travel in a line without rotating.
        """
        # Let the buffer fill before trusting it: the first transform that
        # resolves comes from a partly-populated tree, and planning a straight
        # line from that pose starts from where the arm has never been.
        settle = self.get_clock().now().nanoseconds * 1e-9 + 0.5
        while self.get_clock().now().nanoseconds * 1e-9 < settle:
            rclpy.spin_once(self, timeout_sec=0.05)

        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while self.get_clock().now().nanoseconds * 1e-9 < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    BASE_FRAME, TOOL_LINK, rclpy.time.Time())
            except Exception:
                continue
            # And insist it is RECENT. A stale transform is worse than none: it
            # describes where the arm used to be, which is exactly the input a
            # Cartesian path must not be given.
            age = (self.get_clock().now().nanoseconds * 1e-9
                   - (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9))
            if age > 1.0:
                continue
            p = Pose()
            p.position.x = tf.transform.translation.x
            p.position.y = tf.transform.translation.y
            p.position.z = tf.transform.translation.z
            p.orientation = tf.transform.rotation
            return p
        sys.exit(f"no TF for {BASE_FRAME} -> {TOOL_LINK}. Is the stack up and the "
                 f"hardware active?")

    avoid_collisions = True

    def plan(self, target, step, jump, velocity, acceleration):
        if not self.plan_srv.wait_for_service(timeout_sec=5.0):
            sys.exit("/compute_cartesian_path is not available - is move_group running?")
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.group_name = PLANNING_GROUP
        req.link_name = TOOL_LINK
        req.waypoints = [target]
        # "Start from where the arm is now". An empty RobotState with is_diff
        # false is read as a fully specified state - every joint at zero - so
        # without this the planner solves 0% of the line from a pose the arm is
        # not in. move_to sets the same flag for the same reason.
        req.start_state.is_diff = True
        # How finely the line is sampled before IK is solved at each point. Too
        # coarse and the "straight" line is a chord through the real path; too
        # fine and the request takes noticeably longer for no gain.
        req.max_step = step
        # Guards against the IK solution leaping between branches - a tiny tool
        # motion asking for a huge joint motion, which near a singularity it will.
        # MoveIt accepts 0.0 to disable this, and on real hardware that is how you
        # get a joint slamming across its range mid-line.
        req.jump_threshold = jump
        req.avoid_collisions = self.avoid_collisions
        req.max_velocity_scaling_factor = velocity
        req.max_acceleration_scaling_factor = acceleration
        fut = self.plan_srv.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        res = fut.result()
        if res is None:
            sys.exit("the Cartesian path service did not answer within 30 s")
        return res

    def execute(self, trajectory):
        if not self.exec_ac.wait_for_server(timeout_sec=5.0):
            sys.exit("/execute_trajectory is not available - is move_group running?")
        goal = ExecuteTrajectory.Goal(trajectory=trajectory)
        fut = self.exec_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False, "execution goal was rejected"
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=120.0)
        out = res_fut.result()
        if out is None:
            return False, "no result from execution within 120 s"
        code = out.result.error_code.val
        return code == 1, f"error_code {code}"


def main():
    ap = argparse.ArgumentParser(
        description="Move the tool in a straight line, in Cartesian space.")
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--pose", nargs=3, type=float, metavar=("X", "Y", "Z"),
                       help=f"absolute {TOOL_LINK} position in {BASE_FRAME}, metres")
    where.add_argument("--relative", nargs=3, type=float, metavar=("DX", "DY", "DZ"),
                       help="offset from where the tool is now, metres. "
                            "`--relative 0 0 -0.05` is 5 cm straight down")
    ap.add_argument("--rpy", nargs=3, type=float, metavar=("R", "P", "Y"),
                    help="tool orientation at the end, radians. Omit to keep the "
                         "current orientation, i.e. travel without rotating")
    ap.add_argument("--step", type=float, default=0.005,
                    help="Cartesian sampling interval, metres (default 5 mm)")
    ap.add_argument("--jump", type=float, default=5.0,
                    help="IK jump threshold; 0 disables it, which allows a joint "
                         "to leap mid-line near a singularity (default 5.0)")
    ap.add_argument("--velocity", type=float, default=0.2)
    ap.add_argument("--acceleration", type=float, default=0.2)
    ap.add_argument("--min-fraction", type=float, default=0.95,
                    help="refuse to execute if less of the line than this was "
                         "solved (default 0.95). A partial path stops short of "
                         "the goal, which for an approach move is the wrong place")
    ap.add_argument("--plan-only", action="store_true")
    ap.add_argument("--no-collisions", action="store_true",
                    help="skip collision checking along the line. A DIAGNOSTIC: if "
                         "a line solves 0%% with checking and 100%% without, the "
                         "start state is in collision and that is the thing to fix")
    a = ap.parse_args()

    rclpy.init()
    node = CartesianMove()
    node.avoid_collisions = not a.no_collisions
    try:
        here = node.tool_pose()
        target = Pose()
        if a.pose:
            target.position.x, target.position.y, target.position.z = a.pose
        else:
            target.position.x = here.position.x + a.relative[0]
            target.position.y = here.position.y + a.relative[1]
            target.position.z = here.position.z + a.relative[2]
        if a.rpy:
            q = quat_from_rpy(*a.rpy)
            (target.orientation.x, target.orientation.y,
             target.orientation.z, target.orientation.w) = q
        else:
            target.orientation = here.orientation

        d = math.dist(
            (here.position.x, here.position.y, here.position.z),
            (target.position.x, target.position.y, target.position.z))
        node.get_logger().info(
            f"straight line from ({here.position.x:.3f}, {here.position.y:.3f}, "
            f"{here.position.z:.3f}) to ({target.position.x:.3f}, "
            f"{target.position.y:.3f}, {target.position.z:.3f}) - {d * 1000:.0f} mm"
            + (", rotating the tool" if a.rpy else ", without rotating"))

        res = node.plan(target, a.step, a.jump, a.velocity, a.acceleration)
        n = len(res.solution.joint_trajectory.points)
        pct = res.fraction * 100.0
        node.get_logger().info(
            f"solved {pct:.1f}% of the line, {n} waypoints "
            f"(error_code {res.error_code.val})")

        if res.fraction < a.min_fraction:
            node.get_logger().error(
                f"only {pct:.1f}% of the line is reachable, below the "
                f"{a.min_fraction * 100:.0f}% required. Executing would stop "
                f"{d * (1 - res.fraction) * 1000:.0f} mm short of the goal. "
                f"Usual causes: the line leaves the workspace, crosses a "
                f"singularity, or is blocked - try a shorter move, or move_to if "
                f"the path shape does not matter.")
            sys.exit(2)

        if a.plan_only:
            node.get_logger().info("plan-only: nothing was moved")
            return

        ok, why = node.execute(res.solution)
        if ok:
            node.get_logger().info(f"SUCCESS: travelled {d * 1000:.0f} mm in a line")
        else:
            node.get_logger().error(f"execution failed: {why}")
            sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
