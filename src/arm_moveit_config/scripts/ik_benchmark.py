#!/usr/bin/env python3
"""Benchmark the configured IK solver against the running move_group.

This is the harness behind the solver choice recorded in config/kinematics.yaml
(KDL: 100% solved, median ~4 ms; pick_ik: 80-92%). Re-run it before changing the
solver, its timeout, a joint limit or a joint zero.

Draw random joint configurations inside the limits, compute the tool pose with
/compute_fk, then ask /compute_ik to recover a configuration for it - seeded
from the SRDF's `ready` state, NOT from the answer. Every target is reachable by
construction, so any failure is the solver's. Seeding from the answer makes
almost any solver look perfect, which is how a broken pick_ik configuration was
first missed.

    ros2 launch arm_moveit_config demo.launch.py rviz:=false
    ros2 run arm_moveit_config ik_benchmark.py [--samples 60] [--timeout 0.5]

Limits come from arm_description/config/joint_parameters.yaml and the seed from
config/arm.srdf, so this always tests the installed configuration.
"""

import argparse
import random
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT = "Slider_39"
GROUP = "arm"
TIP = "tool0"


def load_limits():
    path = Path(get_package_share_directory("arm_description")) / "config" / "joint_parameters.yaml"
    joints = yaml.safe_load(path.read_text())["joints"]
    return ({j: (joints[j]["lower"], joints[j]["upper"]) for j in ARM_JOINTS},
            joints[GRIPPER_JOINT]["lower"])


def load_seed(name="ready"):
    path = Path(get_package_share_directory("arm_moveit_config")) / "config" / "arm.srdf"
    root = ET.parse(path).getroot()
    for state in root.findall("group_state"):
        if state.get("name") == name and state.get("group") == GROUP:
            values = {j.get("name"): float(j.get("value")) for j in state.findall("joint")}
            return [values[j] for j in ARM_JOINTS]
    raise KeyError(f"no group_state '{name}' for group '{GROUP}' in {path}")


def robot_state(arm_values, gripper_value):
    state = RobotState()
    state.joint_state = JointState(name=ARM_JOINTS + [GRIPPER_JOINT],
                                   position=[float(v) for v in arm_values] + [float(gripper_value)])
    return state


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", type=int, default=60)
    ap.add_argument("--timeout", type=float, default=0.5,
                    help="per-call IK timeout in seconds (default 0.5)")
    ap.add_argument("--seed", type=int, default=12345)
    args = ap.parse_args()

    limits, gripper = load_limits()
    seed = load_seed()
    rng = random.Random(args.seed)

    rclpy.init()
    node = Node("ik_benchmark")
    fk = node.create_client(GetPositionFK, "compute_fk")
    ik = node.create_client(GetPositionIK, "compute_ik")
    if not (fk.wait_for_service(timeout_sec=15.0) and ik.wait_for_service(timeout_sec=15.0)):
        print("compute_fk / compute_ik not available -- is move_group running?", file=sys.stderr)
        rclpy.shutdown()
        return 1

    solved, times, failures = 0, [], []
    for _ in range(args.samples):
        target = [rng.uniform(*limits[j]) for j in ARM_JOINTS]

        fk_req = GetPositionFK.Request()
        fk_req.header = Header(frame_id="world")
        fk_req.fk_link_names = [TIP]
        fk_req.robot_state = robot_state(target, gripper)
        future = fk.call_async(fk_req)
        rclpy.spin_until_future_complete(node, future)
        pose = future.result().pose_stamped[0]

        request = PositionIKRequest()
        request.group_name = GROUP
        request.robot_state = robot_state(seed, gripper)
        request.ik_link_name = TIP
        request.pose_stamped = pose
        request.timeout.sec = int(args.timeout)
        request.timeout.nanosec = int((args.timeout % 1.0) * 1e9)
        request.avoid_collisions = False
        ik_req = GetPositionIK.Request()
        ik_req.ik_request = request

        start = time.perf_counter()
        future = ik.call_async(ik_req)
        rclpy.spin_until_future_complete(node, future)
        elapsed_ms = (time.perf_counter() - start) * 1000.0

        code = future.result().error_code.val
        if code == 1:
            solved += 1
            times.append(elapsed_ms)
        else:
            p = pose.pose.position
            failures.append((round(p.x, 3), round(p.y, 3), round(p.z, 3), code))

    node.destroy_node()
    rclpy.shutdown()

    print(f"solved {solved}/{args.samples} = {100.0 * solved / args.samples:.0f}%  "
          f"(seeded from 'ready', timeout {args.timeout} s)")
    if times:
        times.sort()
        p95 = times[max(0, int(len(times) * 0.95) - 1)]
        print(f"solve time: median {times[len(times) // 2]:.1f} ms, "
              f"p95 {p95:.1f} ms, max {times[-1]:.1f} ms")
    if failures:
        print(f"failed targets (x, y, z, error code): {failures[:6]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
