#!/usr/bin/env python3
"""Capture the pose dialled up in RViz and make it the new zero.

The arm's zero should be the pose it stands in with every servo at the centre of
its travel. The CAD export does not know that pose and it cannot be fully solved
for: joint4 and joint6 are rolls about the arm's own axis, so turning them moves
no link origin and an optimiser has nothing to say about them. Dial it up by
eye, then run this.

    # 1. sliders, one per joint, bounded by the real limits
    ros2 launch arm_description display.launch.py

    # 2. drag until the arm looks right at zero, then:
    ros2 run arm_description capture_zero.py            # show what would change
    ros2 run arm_description capture_zero.py --apply    # write it

    # 3. rebuild, and re-run what depends on the zero
    colcon build --packages-select arm_description
    ros2 run arm_moveit_config analyse_self_collision.py   # limits moved

Every joint rotates about its own local +Z, so moving a zero by `delta` is
new_origin = old_origin * Rz(delta): only the rpy of six joint origins changes.
No mesh or limit value changes - though what the limits mean physically does,
which is why the collision matrix must be regenerated. Offsets accumulate in
config/joint_parameters.yaml under zero_offsets_rad.
"""

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Joint origin rpy exactly as the Fusion exporter wrote them. The offsets are
# always expressed relative to THIS, so repeated captures cannot drift.
CAD_RPY = {
    "joint1": (-1.579827,  0.000160,  0.017765),
    "joint2": ( 1.570796,  0.231806,  1.495452),
    "joint3": ( 3.141593,  0.000000,  0.000000),
    "joint4": ( 1.570796,  1.549855, -1.297363),
    "joint5": (-1.567187,  0.478036, -1.418347),
    "joint6": (-1.678871, -1.540667,  1.502969),
}


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def matrix_to_rpy(m):
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    return (math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0]))


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def read_joint_states(timeout=15.0):
    rclpy.init()
    node = Node("capture_zero")
    captured = {}

    def on_msg(msg):
        captured.update(dict(zip(msg.name, msg.position)))

    node.create_subscription(JointState, "joint_states", on_msg, 10)
    deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while not captured and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return captured


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apply", action="store_true",
                    help="write the change; without this it only reports")
    ap.add_argument("--source", type=Path, default=None,
                    help="package source dir (default: infer from the share dir)")
    args = ap.parse_args()

    current = read_joint_states()
    if not current:
        print("No /joint_states heard. Start the sliders with:\n"
              "  ros2 launch arm_description display.launch.py", file=sys.stderr)
        return 1

    missing = [j for j in ARM_JOINTS if j not in current]
    if missing:
        print(f"/joint_states did not carry {missing}", file=sys.stderr)
        return 1

    src = args.source
    if src is None:
        share = Path(get_package_share_directory("arm_description")).resolve()
        src = share  # symlink-install points share/ straight at the source tree
    params_path = src / "config" / "joint_parameters.yaml"
    if not params_path.exists():
        print(f"cannot find {params_path}; pass --source", file=sys.stderr)
        return 1

    params = yaml.safe_load(params_path.read_text())
    previous = params.get("zero_offsets_rad", {j: 0.0 for j in ARM_JOINTS})

    print(f"{'joint':8s} {'current':>10s} {'old offset':>12s} {'new offset':>12s}"
          f"   {'new origin rpy'}")
    updates = {}
    for joint in ARM_JOINTS:
        delta = float(current[joint])
        total = float(previous.get(joint, 0.0)) + delta
        rpy = matrix_to_rpy(rpy_to_matrix(*CAD_RPY[joint]) @ rot_z(total))
        updates[joint] = (total, rpy)
        flag = "" if abs(delta) < 1e-6 else "  <- moved"
        print(f"{joint:8s} {math.degrees(delta):9.3f}d "
              f"{math.degrees(previous.get(joint, 0.0)):11.3f}d "
              f"{math.degrees(total):11.3f}d   "
              f"{rpy[0]: .6f} {rpy[1]: .6f} {rpy[2]: .6f}{flag}")

    if not args.apply:
        print("\n(report only -- pass --apply to write it)")
        return 0

    # 1. joint origin rpy, in whichever file each joint lives in
    for path in [src / "urdf" / "assemblies" / "arm_links.urdf.xacro",
                 src / "urdf" / "arm.urdf.xacro"]:
        text = path.read_text()
        out, cur = [], None
        for line in text.split("\n"):
            m = re.search(r'<joint name="\$\{prefix\}(\w+)"', line)
            if m:
                cur = m.group(1)
            if cur in updates and "<origin " in line and "rpy=" in line:
                r, p, y = updates[cur][1]
                line = re.sub(r'rpy="[^"]*"', f'rpy="{r:.6f} {p:.6f} {y:.6f}"', line)
                cur = None
            out.append(line)
        path.write_text("\n".join(out))
        print(f"updated {path.name}")

    # 2. the recorded offsets
    text = params_path.read_text()
    for joint, (total, _) in updates.items():
        text = re.sub(rf"^(  {joint}:)\s*-?[\d.]+.*$",
                      rf"\1 {total: .6f}      # {math.degrees(total):8.3f} deg",
                      text, flags=re.M)
    params_path.write_text(text)
    print(f"updated {params_path.name}")

    print("\nNow, in order:")
    print("  colcon build --packages-select arm_description")
    print("  ros2 run arm_moveit_config analyse_self_collision.py   # limits moved")
    print("  # then re-derive the 'ready' pose -- see docs/PROJECT.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
