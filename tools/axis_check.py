"""Which way does each joint's +axis point, in world frame, at the home pose?

Answers a question the eye is bad at: whether URDF +q is clockwise or
counter-clockwise seen from above. Pure kinematics from the URDF - no hardware.
"""
import subprocess, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

URDF = (Path(__file__).resolve().parent.parent /
        "src/arm_description/urdf/arm.urdf.xacro")
xml = subprocess.run(["xacro", str(URDF)],
                     capture_output=True, text=True, check=True).stdout
root = ET.fromstring(xml)

def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp,   cp*sr,            cp*cr]])

joints = {}
children = {}
for j in root.findall("joint"):
    name = j.get("name")
    parent = j.find("parent").get("link")
    child = j.find("child").get("link")
    o = j.find("origin")
    rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0, 0, 0]
    ax = j.find("axis")
    axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else None
    joints[name] = dict(parent=parent, child=child, rpy=rpy, axis=axis, type=j.get("type"))
    children[child] = name

# Walk from each revolute joint up to the root, accumulating rotation. At the
# home pose every joint angle is zero, so only the fixed origins contribute.
def world_R_of_link(link):
    R = np.eye(3)
    chain = []
    while link in children:
        jn = children[link]
        chain.append(jn)
        link = joints[jn]["parent"]
    for jn in reversed(chain):
        R = R @ rpy_to_R(*joints[jn]["rpy"])
    return R

print(f"{'joint':10s} {'axis in world at home':28s}  seen from above")
for name, j in joints.items():
    if j["type"] != "revolute":
        continue
    R = world_R_of_link(j["parent"]) @ rpy_to_R(*j["rpy"])
    a = R @ np.array(j["axis"])
    z = a[2]
    sense = ("counter-clockwise" if z > 0 else "clockwise") if abs(z) > 0.3 else \
            "(axis near horizontal - no CW/CCW from above)"
    print(f"{name:10s} [{a[0]:+.3f} {a[1]:+.3f} {a[2]:+.3f}]{'':10s}  +q is {sense}")
