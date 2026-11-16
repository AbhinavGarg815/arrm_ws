#!/usr/bin/env python3
"""Trace one joint's commanded vs measured position at full rate, and say whether
a deviation was physical or a sensing glitch.

The distinction matters and is easy to make: a joint moving at 20 deg/s covers
0.2 deg between 100 Hz samples. A step several degrees wide in a single sample is
not motion - no arm accelerates that fast - it is the measurement dropping out.

    tools/trace.py joint1 25          # record 25 s, then report
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

JOINT = sys.argv[1] if len(sys.argv) > 1 else "joint1"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0

class Trace(Node):
    def __init__(self):
        super().__init__("trace")
        self.t, self.meas, self.ref, self.err = [], [], [], []
        self.create_subscription(JointState, "/joint_states", self.js, 100)
        self.create_subscription(JointTrajectoryControllerState,
                                 "/arm_controller/controller_state", self.cs, 100)
    def js(self, m):
        if JOINT in m.name:
            self.t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            self.meas.append(m.position[m.name.index(JOINT)])
    def cs(self, m):
        if JOINT in m.joint_names and m.error.positions:
            i = m.joint_names.index(JOINT)
            self.ref.append(m.reference.positions[i] if m.reference.positions else 0.0)
            self.err.append(m.error.positions[i])

rclpy.init()
n = Trace()
t0 = n.get_clock().now()
while (n.get_clock().now() - t0).nanoseconds < SECONDS * 1e9:
    rclpy.spin_once(n, timeout_sec=0.05)

t = np.array(n.t)
q = np.degrees(np.array(n.meas))
print(f"\n  {JOINT}: {len(q)} samples over {t[-1]-t[0]:.1f} s")
print(f"    range {q.min():+.2f} .. {q.max():+.2f} deg  (span {q.max()-q.min():.2f})")

d = np.abs(np.diff(q))
dt = np.diff(t)
rate = d / np.maximum(dt, 1e-6)
print(f"    largest step between samples: {d.max():.3f} deg "
      f"({rate.max():.0f} deg/s implied)")
big = np.where(d > 1.0)[0]
if len(big):
    print(f"    {len(big)} step(s) over 1 deg — at "
          + ", ".join(f"t+{t[i]-t[0]:.2f}s {q[i]:+.2f}->{q[i+1]:+.2f}" for i in big[:8]))
    print("    A step this wide in 10 ms is NOT motion: it is the sensing dropping out.")
else:
    print("    no single-sample step over 1 deg — deviations were physical motion")
# Smoothness during motion, which a hold test cannot see: a servo whose speed
# cap far exceeds what the trajectory needs crosses each segment in a micro-jump
# and waits, making motion out of 100 Hz steps. That shows up as velocity ripple
# that does not belong to the trajectory, so: velocity minus its own 5-sample
# moving average, RMS.
moving = np.abs(np.diff(q)) > 0.02
if moving.sum() > 50:
    v = np.diff(q) / np.maximum(np.diff(t), 1e-6)
    k = 5
    smooth = np.convolve(v, np.ones(k) / k, mode="same")
    ripple = v - smooth
    sel = moving
    print(f"    WHILE MOVING: speed {np.abs(v[sel]).mean():.1f} deg/s mean, "
          f"{np.abs(v[sel]).max():.1f} peak")
    print(f"    velocity ripple (RMS about its own mean): {ripple[sel].std():.2f} deg/s"
          f"  — lower is smoother")
else:
    print("    (not enough motion in this window to judge smoothness)")

if n.err:
    e = np.degrees(np.abs(np.array(n.err)))
    print(f"    controller following error: max {e.max():.2f} deg, mean {e.mean():.2f} deg")
n.destroy_node(); rclpy.shutdown()
