"""Record the JointTrajectoryController's own following error during a move.

This is the honest measure of whether the arm keeps up with the plan: the
controller compares where the trajectory says each joint should be RIGHT NOW
against where the hardware reports it is.
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

# Accept the duration from whichever argument is a number, and say so plainly
# otherwise. tools/trace.py takes "<joint> <seconds>" and this one takes just
# "<seconds>"; passing trace.py's argument order to this tool crashed it on a
# float() of "joint1" mid-measurement, losing the run.
_nums = [a for a in sys.argv[1:] if a.replace(".", "", 1).isdigit()]
if len(sys.argv) > 1 and not _nums:
    sys.exit(f"usage: {sys.argv[0]} [seconds]   (this tool reports ALL joints; "
             "for one joint's trace use tools/trace.py <joint> <seconds>)")
SECONDS = float(_nums[0]) if _nums else 30.0

class Rec(Node):
    def __init__(self):
        super().__init__("track_rec")
        self.names, self.err, self.vel = None, [], []
        self.create_subscription(JointTrajectoryControllerState,
                                 "/arm_controller/controller_state", self.cb, 100)
    def cb(self, m):
        if self.names is None:
            self.names = list(m.joint_names)
        if m.error.positions:
            self.err.append(list(m.error.positions))
        if m.reference.velocities:
            self.vel.append(list(m.reference.velocities))

rclpy.init()
n = Rec()
t0 = n.get_clock().now()
while (n.get_clock().now() - t0).nanoseconds < SECONDS * 1e9:
    rclpy.spin_once(n, timeout_sec=0.1)

if not n.err:
    print("  no controller_state samples")
else:
    e = np.degrees(np.abs(np.array(n.err)))
    v = np.degrees(np.abs(np.array(n.vel))) if n.vel else None
    print(f"\n  {len(e)} samples")
    print(f"  {'joint':10s} {'max |err|':>10s} {'mean |err|':>11s}"
          + (f" {'peak |v| cmd':>13s}" if v is not None else ""))
    for i, name in enumerate(n.names):
        line = f"  {name:10s} {e[:, i].max():9.2f}° {e[:, i].mean():10.2f}°"
        if v is not None:
            line += f" {v[:, i].max():12.1f}°/s"
        print(line)
n.destroy_node(); rclpy.shutdown()
