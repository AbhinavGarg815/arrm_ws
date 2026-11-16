#!/usr/bin/env python3
"""Move each joint on its own and report how well it followed.

After a reassembly or a servo swap this answers "which joint is wrong" in one
run, instead of inferring it from a whole trajectory that aborts somewhere.

Each joint is moved by a fixed delta in whichever direction has room inside its
limits, measured, and returned to where it started.

    tools/joint_check.py            # +/-12 deg per joint
    tools/joint_check.py 20         # bigger excursion

Needs the live stack up (tools/start_stack.sh).

Everything runs in one process: shelling out per move meant twelve interpreter
startups and loaded the machine enough to kill the run.

Goals go straight to the JointTrajectoryController, so there is NO collision
checking. That is acceptable only because of what this does - one joint at a
time, a small excursion from a pose the arm is already holding, and straight
back. Do not grow it into a general mover; use arm_demos move_to. The firmware's
own soft limits, tracking fault and load trips still apply underneath.
"""
import sys
import time

import rclpy
import yaml
from pathlib import Path
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEG = 57.29577951
DELTA = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
CFG = Path(__file__).resolve().parent.parent / "src/arm_description/config"
LIMITS = yaml.safe_load(open(CFG / "joint_parameters.yaml"))["joints"]


class Check(Node):
    def __init__(self):
        super().__init__("joint_check")
        self.state = None
        self.create_subscription(JointState, "/joint_states", self._cb, 20)
        self.ac = ActionClient(self, FollowJointTrajectory,
                               "/arm_controller/follow_joint_trajectory")

    def _cb(self, m):
        self.state = dict(zip(m.name, m.position))

    def read(self, timeout=5.0):
        end = time.time() + timeout
        self.state = None
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state and all(j in self.state for j in ARM):
                return [self.state[j] for j in ARM]
        sys.exit("no /joint_states - is the stack up and the hardware active?")

    def goto(self, q, seconds):
        """One trajectory point. Returns False if the controller rejects or aborts."""
        traj = JointTrajectory(joint_names=ARM)
        traj.points = [JointTrajectoryPoint(
            positions=[float(v) for v in q],
            time_from_start=Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)))]
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        if not self.ac.wait_for_server(timeout_sec=5.0):
            sys.exit("arm_controller is not accepting goals - check list_controllers")
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=seconds + 15.0)
        out = res.result()
        return out is not None and out.result.error_code == 0


def preflight():
    """Refuse to run unless the arm is actually active.

    Without this, an inactive controller makes every joint report "ABORTED" and
    the output reads as six broken joints when the truth is one unmet
    precondition - which is exactly how this tool first reported a failed
    activation. One cause, one message.
    """
    import subprocess
    hw = subprocess.run(["ros2", "control", "list_hardware_components"],
                        capture_output=True, text=True, timeout=30).stdout
    if "label=active" not in hw:
        sys.exit("  the hardware is NOT active - nothing would move.\n"
                 "  Check /tmp/arm_stack.log; the usual causes are a servo not\n"
                 "  answering the bus or a board refusing to enable.")
    ctl = subprocess.run(["ros2", "control", "list_controllers"],
                         capture_output=True, text=True, timeout=30).stdout
    if not any(l.startswith("arm_controller") and "active" in l
               for l in ctl.splitlines()):
        sys.exit("  arm_controller is not active - it cannot accept goals.\n"
                 "  Run tools/start_stack.sh and wait for 'ready for goals'.")


rclpy.init()
preflight()
n = Check()
home = n.read()
print("  starting from: " + "  ".join(f"{v*DEG:+6.1f}" for v in home))
print(f"  moving each joint by {DELTA:.0f} deg, one at a time, back each time\n")
print(f"  {'joint':8s} {'dir':>4s} {'commanded':>11s} {'reached':>10s} {'error':>8s}  verdict")

rad = DELTA / DEG
clear = 2.0 / DEG
results = []
for i, name in enumerate(ARM):
    lo, hi = LIMITS[name]["lower"], LIMITS[name]["upper"]
    start = home[i]
    if start + rad < hi - clear:
        target, sign = start + rad, "+"
    elif start - rad > lo + clear:
        target, sign = start - rad, "-"
    else:
        print(f"  {name:8s} {'':>4s}   no room inside its limits "
              f"({lo*DEG:+.0f}..{hi*DEG:+.0f})")
        results.append((name, None))
        continue

    q = list(home)
    q[i] = target
    secs = max(2.0, DELTA / 10.0)
    if not n.goto(q, secs):
        print(f"  {name:8s} {sign:>4s} {target*DEG:+10.2f}° {'ABORTED':>10s}"
              f" {'':>8s}  controller rejected or aborted")
        results.append((name, None))
    else:
        got = n.read()[i]
        err = abs(got - target) * DEG
        print(f"  {name:8s} {sign:>4s} {target*DEG:+10.2f}° {got*DEG:+9.2f}°"
              f" {err:7.2f}°  {'ok' if err < 2.5 else 'POOR'}")
        results.append((name, err))
    n.goto(home, secs)
    time.sleep(0.4)

bad = [nm for nm, e in results if e is None or e > 2.5]
print(f"\n  {len(results) - len(bad)} of {len(results)} joints tracked within 2.5 deg")
if bad:
    print(f"  needs attention: {', '.join(bad)}")
n.destroy_node()
rclpy.shutdown()
