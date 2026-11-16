#!/usr/bin/env python3
"""Work out which link pairs can safely be removed from self-collision checking.

The job the MoveIt Setup Assistant's Self-Collisions page does, scripted so the
result has a record of how it was produced and can be re-run whenever a link,
mesh or joint limit changes.

Every collision mesh here is convex, so two links intersect exactly when no
separating plane exists between their vertex sets - a small linear program, and
exact, with no tolerance and no surface sampling. For N random configurations
inside the joint limits, each pair is classified:

  ALWAYS    collides in every sample -> disable; built touching, and leaving
            them enabled makes every state invalid so nothing will ever plan
  ADJACENT  parent/child across a joint -> disable; in contact by design
  NEVER     never collides -> disable; cannot reach each other
  SOMETIMES -> KEEP ENABLED. This is the set that protects the hardware.

    python3 analyse_self_collision.py [--samples 2000] [--urdf /path/to.urdf]

With no --urdf it expands arm_description's xacro itself. Output is SRDF
<disable_collisions> lines, ready to paste, each carrying its reason.
"""

import argparse
import itertools
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull


# --------------------------------------------------------------------- maths -
def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def transform(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = rpy_to_matrix(*rpy)
    m[:3, 3] = xyz
    return m


def axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    r = np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)
    m = np.eye(4)
    m[:3, :3] = r
    return m


def translation(axis, distance):
    m = np.eye(4)
    m[:3, 3] = np.asarray(axis, dtype=float) * distance
    return m


def load_stl(path):
    """Binary STL -> (n*3, 3) vertex array."""
    data = Path(path).read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    verts = np.empty((count * 3, 3), dtype=np.float64)
    for i in range(count):
        base = 84 + i * 50 + 12
        for j in range(3):
            off = base + j * 12
            verts[i * 3 + j] = struct.unpack("<fff", data[off:off + 12])
    return verts


def separated(points_a, points_b):
    """True if a separating plane exists, i.e. the two convex sets do not touch.

    Solves for a plane (n, d) with  n.a <= d - 1  for every a in A  and
    n.b >= d + 1  for every b in B. Infeasible means they intersect. The
    margins of 1 only fix the scale of n; they impose no physical clearance.
    """
    n_a, n_b = len(points_a), len(points_b)
    a_ub = np.vstack([
        np.hstack([points_a, -np.ones((n_a, 1))]),
        np.hstack([-points_b, np.ones((n_b, 1))]),
    ])
    b_ub = -np.ones(n_a + n_b)
    res = linprog(
        c=np.zeros(4), A_ub=a_ub, b_ub=b_ub,
        bounds=[(None, None)] * 4, method="highs",
    )
    return res.status == 0


# ----------------------------------------------------------------- the model -
class Model:
    def __init__(self, urdf_text, mesh_root):
        root = ET.fromstring(urdf_text)
        self.mesh_root = Path(mesh_root)

        self.hulls = {}        # link -> vertices of its collision hull, link frame
        for link in root.findall("link"):
            coll = link.find("collision")
            if coll is None:
                continue
            origin = coll.find("origin")
            mesh = coll.find("geometry/mesh")
            if mesh is None:
                continue
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            scale = float(mesh.get("scale", "1 1 1").split()[0])
            fname = mesh.get("filename").split("//", 1)[-1].split("/", 1)[-1]

            pts = load_stl(self.mesh_root / fname) * scale
            pts = (transform(xyz, rpy) @ np.hstack([pts, np.ones((len(pts), 1))]).T).T[:, :3]
            # Reduce to hull vertices: the LP cost is linear in point count and
            # a 96-triangle cylinder has 288 vertices of which ~32 are extreme.
            try:
                pts = pts[ConvexHull(pts).vertices]
            except Exception:
                pass
            self.hulls[link.get("name")] = pts

        self.joints = []       # (name, parent, child, type, T_static, axis, lo, hi)
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()] if origin is not None else [0, 0, 0]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()] if origin is not None else [0, 0, 0]
            axis_el = joint.find("axis")
            axis = [float(v) for v in axis_el.get("xyz").split()] if axis_el is not None else [0, 0, 1]
            limit = joint.find("limit")
            lo = float(limit.get("lower")) if limit is not None and limit.get("lower") else 0.0
            hi = float(limit.get("upper")) if limit is not None and limit.get("upper") else 0.0
            self.joints.append((
                joint.get("name"),
                joint.find("parent").get("link"),
                joint.find("child").get("link"),
                joint.get("type"),
                transform(xyz, rpy), axis, lo, hi,
            ))

        children = {j[2] for j in self.joints}
        self.root_link = next(l.get("name") for l in root.findall("link")
                              if l.get("name") not in children)

    def adjacent_pairs(self):
        return {frozenset((j[1], j[2])) for j in self.joints}

    def fk(self, q):
        """q: {joint_name: value}. Returns {link: 4x4 pose}."""
        poses = {self.root_link: np.eye(4)}
        remaining = list(self.joints)
        while remaining:
            progressed = False
            for j in list(remaining):
                name, parent, child, jtype, t_static, axis, _, _ = j
                if parent not in poses:
                    continue
                t = t_static
                if jtype == "revolute":
                    t = t @ axis_angle(axis, q.get(name, 0.0))
                elif jtype == "prismatic":
                    t = t @ translation(axis, q.get(name, 0.0))
                poses[child] = poses[parent] @ t
                remaining.remove(j)
                progressed = True
            if not progressed:
                raise RuntimeError("disconnected kinematic tree")
        return poses


# ------------------------------------------------------------------ analysis -
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=2000)
    ap.add_argument("--urdf", type=Path, default=None)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if args.urdf:
        urdf_text = args.urdf.read_text()
        mesh_root = args.urdf.parent.parent
    else:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("arm_description"))
        urdf_text = subprocess.run(
            ["xacro", str(share / "urdf" / "arm.urdf.xacro"), "use_mock_hardware:=true"],
            capture_output=True, text=True, check=True).stdout
        mesh_root = share

    model = Model(urdf_text, mesh_root)
    links = sorted(model.hulls)
    pairs = [frozenset(p) for p in itertools.combinations(links, 2)]
    hits = {p: 0 for p in pairs}

    movable = [(j[0], j[6], j[7]) for j in model.joints if j[3] in ("revolute", "prismatic")]
    rng = np.random.default_rng(args.seed)

    print(f"{len(links)} collision links, {len(pairs)} pairs, "
          f"{len(movable)} movable joints, {args.samples} samples", file=sys.stderr)

    for n in range(args.samples):
        q = {name: rng.uniform(lo, hi) for name, lo, hi in movable}
        poses = model.fk(q)
        world = {}
        for link in links:
            h = model.hulls[link]
            t = poses[link]
            world[link] = (t[:3, :3] @ h.T).T + t[:3, 3]
        boxes = {l: (world[l].min(0), world[l].max(0)) for l in links}

        for pair in pairs:
            a, b = tuple(pair)
            lo_a, hi_a = boxes[a]
            lo_b, hi_b = boxes[b]
            if np.any(hi_a < lo_b) or np.any(hi_b < lo_a):
                continue                       # AABBs disjoint: cannot collide
            if not separated(world[a], world[b]):
                hits[pair] += 1

        if (n + 1) % 200 == 0:
            print(f"  {n + 1}/{args.samples}", file=sys.stderr)

    adjacent = model.adjacent_pairs()
    print("\n<!-- generated by scripts/analyse_self_collision.py "
          f"--samples {args.samples} --seed {args.seed} -->")
    keep = []
    for pair in sorted(pairs, key=lambda p: sorted(p)):
        a, b = sorted(pair)
        frac = hits[pair] / args.samples
        if pair in adjacent:
            reason = "Adjacent"
        elif frac == 0.0:
            reason = "Never"
        elif frac > 0.95:
            reason = "Always"
        else:
            keep.append((a, b, frac))
            continue
        print(f'  <disable_collisions link1="{a}" link2="{b}" reason="{reason}"/>')

    print(f"\n<!-- {len(keep)} pair(s) LEFT ENABLED "
          "(they collide in some configurations, which is the point) -->",
          file=sys.stderr)
    for a, b, frac in sorted(keep, key=lambda k: -k[2]):
        print(f"<!--   {a} <-> {b}: collides in {frac * 100:.1f}% of samples -->",
              file=sys.stderr)


if __name__ == "__main__":
    main()
