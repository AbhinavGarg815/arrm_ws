#!/usr/bin/env python3
"""Map what the arm can actually reach, and where it can point the tool down.

"Plan to a goal pose" fails either because the position is out of reach or
because the arm cannot hold the tool at that orientation there, and the error
code does not distinguish them. The second is the common one here, with joint4,
joint5 and joint2 all limited to +/-90 deg: a point 23 mm from `ready` has no IK
solution at `ready`'s own orientation.

Forward sampling, not IK: draw random configurations inside the limits, reject
the self-colliding ones, and record where tool0 ends up and which way it points.
Every recorded pose is reachable by construction, so there is no solver to blame
and no timeout to tune. Results are binned in the vertical plane through the arm
(radius from the base axis, height), because the base joint rotates the whole
workspace and the map is very nearly a surface of revolution.

    python3 workspace_map.py [--samples 20000] [--max-tilt 20]
"""

import argparse
import itertools
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyse_self_collision import Model, separated  # noqa: E402

JOINT_LIMITS = {
    "joint1": (-2.094395, 2.094395),
    "joint2": (-1.570796, 1.570796),
    "joint3": (-2.094395, 2.094395),
    "joint4": (-1.570796, 1.570796),
    "joint5": (-1.570796, 1.570796),
    "joint6": (-3.141593, 3.141593),
}


def load_model():
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory("arm_description"))
    urdf = subprocess.run(
        ["xacro", str(share / "urdf" / "arm.urdf.xacro"), "use_mock_hardware:=true"],
        capture_output=True, text=True, check=True).stdout
    return Model(urdf, share)


def self_colliding(model, links, adjacent, poses):
    world = {}
    for link in links:
        t = poses[link]
        world[link] = (t[:3, :3] @ model.hulls[link].T).T + t[:3, 3]
    boxes = {l: (world[l].min(0), world[l].max(0)) for l in links}
    for a, b in itertools.combinations(links, 2):
        if frozenset((a, b)) in adjacent:
            continue
        lo_a, hi_a = boxes[a]
        lo_b, hi_b = boxes[b]
        if np.any(hi_a < lo_b) or np.any(hi_b < lo_a):
            continue
        if not separated(world[a], world[b]):
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=20000)
    ap.add_argument("--max-tilt", type=float, default=20.0,
                    help="degrees from straight-down still counted as 'can point down'")
    ap.add_argument("--check-collisions", action="store_true",
                    help="reject self-colliding samples (slow, but honest)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    model = load_model()
    links = sorted(model.hulls)
    adjacent = model.adjacent_pairs()
    rng = np.random.default_rng(args.seed)

    reach, down = [], []
    rejected = 0
    for i in range(args.samples):
        q = {name: rng.uniform(lo, hi) for name, (lo, hi) in JOINT_LIMITS.items()}
        q["Slider_39"] = -0.065
        poses = model.fk(q)
        if args.check_collisions and self_colliding(model, links, adjacent, poses):
            rejected += 1
            continue
        t = poses["tool0"]
        p = t[:3, 3]
        approach = t[:3, :3] @ np.array([0.0, 0.0, 1.0])
        tilt = np.degrees(np.arccos(np.clip(-approach[2], -1, 1)))
        radius = float(np.hypot(p[0], p[1]))
        reach.append((radius, p[2]))
        if tilt <= args.max_tilt:
            down.append((radius, p[2]))
        if args.check_collisions and (i + 1) % 500 == 0:
            print(f"  {i + 1}/{args.samples}", file=sys.stderr)

    reach = np.array(reach)
    down = np.array(down) if down else np.empty((0, 2))

    print(f"samples: {len(reach)} valid"
          + (f", {rejected} rejected as self-colliding" if args.check_collisions else "")
          + f"\ntool0 reach: radius {reach[:, 0].min():.3f}-{reach[:, 0].max():.3f} m, "
          f"height {reach[:, 1].min():.3f}-{reach[:, 1].max():.3f} m")
    print(f"can point within {args.max_tilt:.0f} deg of straight down: "
          f"{len(down)} of {len(reach)} samples ({100 * len(down) / len(reach):.1f}%)\n")

    # occupancy grid, 5 cm cells, in the (radius, height) plane
    cell = 0.05
    r_edges = np.arange(0, reach[:, 0].max() + cell, cell)
    z_edges = np.arange(0, reach[:, 1].max() + cell, cell)
    total, _, _ = np.histogram2d(reach[:, 0], reach[:, 1], bins=[r_edges, z_edges])
    if len(down):
        vertical, _, _ = np.histogram2d(down[:, 0], down[:, 1], bins=[r_edges, z_edges])
    else:
        vertical = np.zeros_like(total)

    print("Vertical slice. Rows = height above ground, columns = radius from the")
    print("base axis, both in metres. Each cell shows what the tool can do there:")
    print(f"  #  can point within {args.max_tilt:.0f} deg of straight down")
    print("  +  reachable, but only at other orientations")
    print("  .  not reachable\n")
    header = "        " + "".join(f"{r:<5.2f}" for r in r_edges[:-1])
    print(header)
    for zi in range(len(z_edges) - 2, -1, -1):
        row = f"  {z_edges[zi]:5.2f} "
        for ri in range(len(r_edges) - 1):
            if vertical[ri, zi] > 0:
                row += "#    "
            elif total[ri, zi] > 0:
                row += "+    "
            else:
                row += ".    "
        print(row)
    print("\n(radius, not x: the base rotates +/-120 deg so the workspace is very")
    print(" nearly a surface of revolution about the vertical axis.)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
