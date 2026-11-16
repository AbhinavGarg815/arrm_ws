#!/usr/bin/env python3
"""Per-joint smoothness DURING motion, for every joint at once.

A hold test cannot see this. Two different faults show up here:
  * a joint vibrating in place while the REST of the arm moves - a servo loop
    poorly damped against the inertia hanging off it;
  * a joint crossing its path in 100 Hz micro-jumps - a servo whose own speed cap
    is far above what the trajectory needs, so it darts to each goal and waits.

Ripple is the velocity minus its own 5-sample moving average, RMS, over the
samples where the joint was actually moving OR the arm was. Lower is smoother.
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 22.0

class Rec(Node):
    def __init__(self):
        super().__init__("moving_ripple")
        self.t, self.q, self.names = [], [], None
        self.create_subscription(JointState, "/joint_states", self.cb, 100)
    def cb(self, m):
        if self.names is None:
            self.names = list(m.name)
        self.t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.q.append(list(m.position))

rclpy.init()
n = Rec()
t0 = n.get_clock().now()
while (n.get_clock().now() - t0).nanoseconds < SECONDS * 1e9:
    rclpy.spin_once(n, timeout_sec=0.05)

if len(n.t) < 3:
    # Crashing with "zero-size array to reduction" tells the operator nothing
    # about what is wrong. The usual cause is the hardware having deactivated
    # mid-run, which stops /joint_states.
    sys.exit(f"  only {len(n.t)} sample(s) of /joint_states - the arm is probably\n"
             f"  not active. Check:  ros2 control list_controllers\n"
             f"  and:  grep -E 'fault on|sagging' /tmp/arm_stack.log | tail -3")

t = np.array(n.t)
q = np.degrees(np.array(n.q))
dt = np.maximum(np.diff(t), 1e-6)
v = np.diff(q, axis=0) / dt[:, None]

# "The arm is moving" = any joint above 1 deg/s. Judge every joint over that
# same window, so a joint that should be still is measured while the arm shakes it.
arm_moving = np.abs(v).max(axis=1) > 1.0
print(f"\n  {len(t)} samples, {arm_moving.sum()} of {len(v)} while the arm moved\n")
print(f"  {'joint':10s} {'travel':>8s} {'peak |v|':>9s} {'ripple':>9s}  verdict")
for i, name in enumerate(n.names):
    x, vi = q[:, i], v[:, i]
    if not np.all(np.isfinite(x)) or arm_moving.sum() < 30:
        print(f"  {name:10s}  —"); continue
    k = 5
    ripple = (vi - np.convolve(vi, np.ones(k) / k, mode="same"))[arm_moving]
    travel = x.max() - x.min()
    r = ripple.std()
    # A joint that hardly travels but ripples hard is being shaken, not driven.
    verdict = ("vibrating in place" if travel < 5.0 and r > 5.0 else
               "rough" if r > 10.0 else "smooth")
    print(f"  {name:10s} {travel:7.2f}° {np.abs(vi[arm_moving]).max():8.1f}°/s "
          f"{r:8.2f}°/s  {verdict}")
n.destroy_node(); rclpy.shutdown()
