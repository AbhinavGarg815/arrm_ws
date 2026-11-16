#!/usr/bin/env python3
"""Make the arm's CURRENT pose the zero, by recording where each motor sits.

Use when the arm was reassembled and does not look like the model does at zero.
Dial the arm up by eye until it IS the zero pose (RViz MotionPlanning -> Joints
tab, or tools/motor_test.py), then run this.

    tools/set_zeros.py            # show what would change
    tools/set_zeros.py --apply    # write it to actuators.yaml

Not capture_zero.py: that moves the URDF's zero (the joint origins) and is for
when the CAD pose itself is wrong. For a MOUNTING offset it is the wrong
instrument - it would redefine the model to match however the links were bolted
on, change what every joint limit means physically, and force the self-collision
matrix to be regenerated. A mounting offset belongs in actuators.yaml, which
exists to say where the motor sits when the URDF joint reads zero.

The raw motor position is recovered from the published angle and the mapping
currently in force:

    Mega   motor_deg = zero_deg    + direction * q * 180/pi
    ESP32  counts    = zero_counts + direction * q * counts_per_rad

Those become the new zeros, so this pose reads q = 0 afterwards. Needs the live
stack up, because it reads /joint_states.
"""
import argparse
import math
import re
import sys
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

ACT = (Path(__file__).resolve().parent.parent /
       "src/arm_description/config/actuators.yaml")
DEG = 180.0 / math.pi


def live_state(seconds=3.0):
    """Mean and spread of each joint over a few seconds.

    Averaged on purpose. A single sample bakes in whatever the arm happened to be
    doing at that instant - and these joints droop and settle inside their own
    backlash while holding, which is motion, not a mounting offset. The spread is
    printed so a pose that has not settled is visible rather than silently
    captured.
    """
    import time
    rclpy.init()
    n = Node("set_zeros")
    samples = {}

    def cb(m):
        for name, pos in zip(m.name, m.position):
            samples.setdefault(name, []).append(pos)

    n.create_subscription(JointState, "/joint_states", cb, 50)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()
    if not samples:
        sys.exit("no /joint_states - bring the stack up first (tools/start_stack.sh)")
    return ({k: sum(v) / len(v) for k, v in samples.items()},
            {k: (max(v) - min(v)) for k, v in samples.items()})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--joints", nargs="+", metavar="JOINT",
                    help="only these joints (e.g. --joints joint3 joint5). Without "
                         "it, every joint that differs is captured - which also "
                         "captures droop and backlash on the joints you did not "
                         "mean to touch. Name the ones you deliberately posed.")
    a = ap.parse_args()

    cfg = yaml.safe_load(ACT.read_text())["joints"]
    q, spread = live_state()

    print(f"  {'joint':10s} {'q mean':>9s} {'p-p':>7s} {'board':7s} "
          f"{'zero now':>10s} {'zero new':>10s}   change")
    changes = {}
    for name, c in cfg.items():
        board = c.get("board")
        if board not in ("mega", "esp32") or name not in q:
            continue
        if a.joints and name not in a.joints:
            continue
        d = float(c["direction"])
        if board == "mega":
            old = float(c.get("motor_angle_at_zero_deg", 0.0))
            new = old + d * q[name] * DEG
            key, unit, shown = "motor_angle_at_zero_deg", "deg", f"{new:10.2f}"
            moved = f"{(new - old):+.2f} deg"
        else:
            old = float(c.get("motor_counts_at_zero", 2048))
            cpr = float(c.get("counts_per_rad", 651.899))
            new = round(old + d * q[name] * cpr)
            key, unit, shown = "motor_counts_at_zero", "ct", f"{new:10d}"
            dct = int(new - round(old))
            moved = f"{dct:+d} ct = {dct / cpr * DEG:+.2f} deg"
        print(f"  {name:10s} {q[name]*DEG:+8.2f}° {spread[name]*DEG:6.2f}° "
              f"{board:7s} {old:10.2f} {shown}   {moved}")
        # Only worth writing if it actually moves the zero.
        if abs(new - old) > (0.02 if board == "mega" else 1):
            changes[name] = (key, new, unit)

    small = [n for n, (_, nv, u) in changes.items()
             if abs(q[n] * DEG) < 2.0]
    if small:
        print(f"\n  CAUTION: {', '.join(small)} are within 2 deg of zero already.")
        print("  These joints droop and settle inside their own backlash while")
        print("  holding (0.52-1.08 deg, plus gravity), so a change that small is")
        print("  more likely to be that than a mounting offset - and capturing it")
        print("  moves the zero by whatever the arm was doing at the time.")
        print("  Capture a joint only when it is VISIBLY off at the zero pose.")

    if not changes:
        print("\n  nothing to change - the arm is already at its zero")
        return
    if not a.apply:
        print(f"\n  {len(changes)} joint(s) would change. Re-run with --apply to write.")
        return

    text = ACT.read_text()
    for name, (key, new, unit) in changes.items():
        # Replace the value inside this joint's block only.
        m = re.search(rf"(^  {name}:\n(?:.*\n)*?\s*{key}:\s*)([-0-9.]+)",
                      text, re.M)
        if not m:
            sys.exit(f"could not find {key} under {name} in actuators.yaml")
        val = f"{new:.2f}" if unit == "deg" else f"{new}"
        text = text[:m.start(2)] + val + text[m.end(2):]
    ACT.write_text(text)
    print(f"\n  wrote {len(changes)} zero(s) to {ACT.name}")
    print("  now: colcon build --packages-select arm_description --symlink-install")
    print("       then restart the stack; this pose should read q = 0 for every joint.")
    print("  NOTE: joint limits are measured from the zero, so a large change here")
    print("        means the soft limits are now offset by the same amount. Check")
    print("        joint_parameters.yaml against the real stops before driving far.")


main()
