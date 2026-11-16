#!/usr/bin/env python3
"""Audit, and regenerate, the collision geometry as convex hulls of the visuals.

The Fusion exporter writes one axis-aligned box per link, and those boxes do not
contain the parts they stand for. Measured by testing every visual vertex
against the collision hull's half-spaces:

    link              visual geometry outside its box      worst protrusion
    Link1_Shoulder                 62.9 %                        107.0 mm
    Link6_Gripper                  48.7 %                         99.7 mm
    Link3_ForeArm                  32.6 %                         66.3 mm
    Link5_Wrist2                   30.3 %                         43.4 mm
    Link2_UpperArm                 30.1 %                         96.3 mm
    Link4_Wrist1                   20.3 %                         47.8 mm
    base_link                       0.1 %                          0.4 mm
    Finger_Moving                   0.0 %                          0.0 mm

Geometry that small does not make the planner conservative, it makes it wrong in
the dangerous direction: MoveIt reports a path clear while the links pass
through each other. So this audits every link and, for those that fail, writes
the convex hull of the visual mesh as a binary STL - a hull can never
under-cover the part and is cheap for FCL to test.

Gripper caveat, before any grasp work: a convex hull of Link6_Gripper bridges
the gap between the jaws. Safe while the task is moving without hitting
anything, wrong the moment you want to grasp, because an object between the
fingers reads as a collision and no grasp will ever plan. Replace it with a
convex decomposition (VHACD / CoACD) or hand-authored primitives first.

    python3 scripts/make_collision_meshes.py --audit   # report only
    python3 scripts/make_collision_meshes.py           # regenerate what fails
"""

import argparse
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}

# A link whose visual geometry pokes further than this out of its declared
# collision geometry gets regenerated. 1 mm is below the arm's own backlash
# (0.52-1.08 deg, i.e. several mm at the tip), so anything under it is noise.
TOLERANCE_M = 0.001


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


def apply(t, pts):
    return (t[:3, :3] @ pts.T).T + t[:3, 3]


def dae_vertices(path: Path) -> np.ndarray:
    """Every vertex position in a COLLADA file, in the mesh's own frame."""
    root = ET.parse(path).getroot()
    unit = root.find("c:asset/c:unit", COLLADA_NS)
    scale = float(unit.get("meter", "1.0")) if unit is not None else 1.0
    chunks = [
        np.fromstring(a.text, sep=" ").reshape(-1, 3)
        for a in root.iter(f"{{{COLLADA_NS['c']}}}float_array")
        if "positions" in (a.get("id") or "")
    ]
    if not chunks:
        raise ValueError(f"no position arrays in {path}")
    return np.vstack(chunks) * scale


def stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    out = np.empty((count * 3, 3))
    for i in range(count):
        base = 84 + i * 50 + 12
        for j in range(3):
            out[i * 3 + j] = struct.unpack("<fff", data[base + j * 12:base + j * 12 + 12])
    return out


def write_binary_stl(path: Path, points: np.ndarray, faces: np.ndarray) -> None:
    header = b"convex hull - arm_description/scripts/make_collision_meshes.py"
    with path.open("wb") as fh:
        fh.write(header.ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(faces)))
        for face in faces:
            a, b, c = points[face[0]], points[face[1]], points[face[2]]
            n = np.cross(b - a, c - a)
            mag = np.linalg.norm(n)
            fh.write(struct.pack("<3f", *(n / mag if mag > 0 else np.zeros(3))))
            for v in (a, b, c):
                fh.write(struct.pack("<3f", *v))
            fh.write(struct.pack("<H", 0))


def protrusion(visual_pts: np.ndarray, collision_pts: np.ndarray):
    """(fraction outside, worst distance outside) of visual vs collision hull."""
    hull = ConvexHull(collision_pts)
    normals, offsets = hull.equations[:, :3], hull.equations[:, 3]
    signed = (visual_pts @ normals.T + offsets).max(axis=1)
    return float((signed > 1e-6).mean()), float(signed.max())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--audit", action="store_true", help="report only, write nothing")
    ap.add_argument("--package-root", type=Path,
                    default=Path(__file__).resolve().parent.parent)
    ap.add_argument("--urdf", default="urdf/arm_final.urdf",
                    help="flat URDF to audit, relative to the package root")
    args = ap.parse_args()

    root = args.package_root
    robot = ET.parse(root / args.urdf).getroot()

    def geometry(element):
        origin = element.find("origin")
        mesh = element.find("geometry/mesh")
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        scale = float(mesh.get("scale", "1 1 1").split()[0])
        rel = Path(mesh.get("filename").split("//", 1)[-1].split("/", 1)[-1])
        return transform(xyz, rpy), scale, rel

    print(f"{'link':16s} {'outside':>9s} {'worst':>10s}   action")
    regenerated = []
    for link in robot.findall("link"):
        name = link.get("name")
        visual, collision = link.find("visual"), link.find("collision")
        if visual is None or collision is None:
            continue

        t_v, s_v, f_v = geometry(visual)
        t_c, s_c, f_c = geometry(collision)
        vis = apply(t_v, dae_vertices(root / f_v) * s_v)
        col = apply(t_c, stl_vertices(root / f_c) * s_c)

        frac, worst = protrusion(vis, col)
        if worst <= TOLERANCE_M:
            print(f"{name:16s} {frac * 100:8.1f}% {worst * 1000:8.1f} mm   ok, kept")
            continue

        if args.audit:
            print(f"{name:16s} {frac * 100:8.1f}% {worst * 1000:8.1f} mm   FAILS")
            continue

        # Hull is built in the VISUAL MESH's own frame, so the new <collision>
        # entry uses the same <origin> as the <visual> and scale="1 1 1".
        raw = dae_vertices(root / f_v) * s_v
        hull = ConvexHull(raw)
        verts = raw[hull.vertices]
        remap = {old: new for new, old in enumerate(hull.vertices)}
        faces = np.array([[remap[i] for i in s] for s in hull.simplices])

        out = (root / f_v).with_name(f"{name}_convex.stl")
        write_binary_stl(out, verts, faces)
        regenerated.append((name, out.relative_to(root), f_v, len(faces)))
        print(f"{name:16s} {frac * 100:8.1f}% {worst * 1000:8.1f} mm   "
              f"-> {out.name} ({len(faces)} tris)")

    if args.audit:
        print("\n(audit only — nothing written)")
        return 0

    if regenerated:
        print("\nPoint each <collision> at its new hull, using the SAME <origin> as")
        print("the matching <visual> and scale=\"1 1 1\":\n")
        for name, out, vis, tris in regenerated:
            print(f'  {name}: package://arm_description/{out}')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
