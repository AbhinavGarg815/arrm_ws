"""Measure per-joint jitter while the arm holds, from /joint_states.

Safe to run with the stack live - it only subscribes. Reports peak-to-peak and
sd in degrees, plus the dominant frequency, which is what distinguishes a
control limit cycle (sub-Hz) from servo deadband dither (tens of Hz).
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

class Rec(Node):
    def __init__(self):
        super().__init__("jitter_rec")
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
    rclpy.spin_once(n, timeout_sec=0.1)

if len(n.t) < 3:
    # An IndexError on t[-1] tells the operator nothing. The usual cause is the
    # hardware not being active, which stops /joint_states.
    sys.exit(f"  only {len(n.t)} sample(s) of /joint_states - the arm is probably\n"
             f"  not active. Check:  ros2 control list_controllers\n"
             f"  and:  grep -E 'fault on|sagging' /tmp/arm_stack.log | tail -3")

t = np.array(n.t); q = np.degrees(np.array(n.q))
print(f"  {len(t)} samples over {t[-1]-t[0]:.1f} s "
      f"({len(t)/(t[-1]-t[0]):.0f} Hz)\n")
print(f"  {'joint':10s} {'mean':>9s} {'p-p':>7s} {'sd':>7s} {'drift':>7s}  dominant")
for i, name in enumerate(n.names):
    x = q[:, i]
    if not np.all(np.isfinite(x)):
        print(f"  {name:10s}  non-finite"); continue
    xd = x - x.mean()
    # Dominant frequency of the wobble, ignoring DC.
    fs = len(t) / (t[-1] - t[0])
    sp = np.abs(np.fft.rfft(xd * np.hanning(len(xd))))
    fr = np.fft.rfftfreq(len(xd), 1 / fs)
    k = np.argmax(sp[1:]) + 1
    print(f"  {name:10s} {x.mean():+9.3f} {x.max()-x.min():7.3f} {x.std():7.3f} "
          f"{x[-1]-x[0]:+7.3f}  {fr[k]:5.2f} Hz")
n.destroy_node(); rclpy.shutdown()
