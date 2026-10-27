#!/usr/bin/env python3
"""
Predict the smallest passable corridor gap versus wrist roll from the COLLISION MESHES alone
(no ROS, no MoveIt), and say which link limits it.

For each roll it places the robot at the shared goal pose, samples points on every collision mesh, keeps the points
inside the corridor's x-z slab (the wall boxes), and takes the largest |y|. With the robot centred on y = 0,
  predicted gap = 2 * max |y|  over all points in the slab.
Compare with clearance_curve.py's measurement: close agreement means the "effective cross-section" explanation
is measured, not assumed.

  python3 mesh_width.py urdf/robot_6dof.urdf \
      --mesh-dir ~/new_ws_review/src/open_manipulator/open_manipulator_description/meshes/open_manipulator_x \
      --config 6dof --compare ~/analysis/clearance_6dof.csv --out ~/analysis/mesh_width_6dof.csv

Needs numpy and fk_check.py in the same folder. The URDF must be an expanded URDF (see wrap_6dof.urdf.xacro).
Slab defaults match scene_builder_variable.py: walls 0.15 deep at x = 0.30 -> x in [0.225, 0.375]; z in [0, 0.4].
"""
import argparse
import csv
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk_check import rpy_to_R, axis_angle_R, T_from  # noqa: E402

BASE = {"6dof": {"joint1": 0.0, "joint2": 0.09061371837047713, "joint3": 0.19387853686569093,
                 "joint4": -0.28449225523616806},
        "5dof": {"joint1": 0.0, "joint2": 0.34549575828472284, "joint3": -0.1686695073991569,
                 "joint4": -0.1768262508855657}}


def read_stl(path):
    data = open(path, "rb").read()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
            return np.frombuffer(data, dtype=rec, count=n, offset=84)["v"].astype(float)
    txt = data.decode("utf-8", errors="ignore")
    v = re.findall(r"vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", txt)
    if not v:
        sys.exit(f"cannot read STL: {path}")
    return np.array(v, dtype=float).reshape(-1, 3, 3)


def sample_surface(tris, n, rng):
    a, b = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(a, b), axis=1)
    if area.sum() <= 0:
        return tris.reshape(-1, 3)
    idx = rng.choice(len(tris), size=n, p=area / area.sum())
    u, v = rng.random((n, 1)), rng.random((n, 1))
    flip = (u + v) > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    pts = tris[idx, 0] + u * a[idx] + v * b[idx]
    return np.vstack([pts, tris.reshape(-1, 3)])


def floats(s, d):
    return [float(x) for x in s.split()] if s else d


def parse(urdf, mesh_dir, n_samples, rng):
    root = ET.parse(urdf).getroot()
    joints = {}
    for j in root.findall("joint"):
        o, ax = j.find("origin"), j.find("axis")
        joints[j.find("child").get("link")] = dict(
            name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
            xyz=floats(o.get("xyz") if o is not None else None, [0, 0, 0]),
            rpy=floats(o.get("rpy") if o is not None else None, [0, 0, 0]),
            axis=floats(ax.get("xyz") if ax is not None else None, [1, 0, 0]))
    links = [l.get("name") for l in root.findall("link")]
    roots = [l for l in links if l not in joints]
    bodies = {}
    for l in root.findall("link"):
        col = l.find("collision")
        if col is None:
            continue
        mesh = col.find("geometry/mesh")
        if mesh is None:
            print(f"note: link {l.get('name')} has a non-mesh collision geometry: ignored")
            continue
        fn = os.path.basename(mesh.get("filename"))
        path = os.path.join(os.path.expanduser(mesh_dir), fn)
        if not os.path.exists(path):
            sys.exit(f"mesh not found: {path}")
        scale = np.array(floats(mesh.get("scale"), [1, 1, 1]))
        o = col.find("origin")
        Tc = T_from(rpy_to_R(*floats(o.get("rpy") if o is not None else None, [0, 0, 0])),
                    floats(o.get("xyz") if o is not None else None, [0, 0, 0]))
        pts = sample_surface(read_stl(path) * scale, n_samples, rng)
        bodies[l.get("name")] = (pts, Tc, fn)
    return joints, roots, bodies


def link_poses(joints, roots, q):
    T = {r: np.eye(4) for r in roots}
    children = {}
    for child, j in joints.items():
        children.setdefault(j["parent"], []).append(child)
    stack = list(roots)
    while stack:
        p = stack.pop()
        for c in children.get(p, []):
            j = joints[c]
            Tj = T[p] @ T_from(rpy_to_R(*j["rpy"]), j["xyz"])
            val = q.get(j["name"], 0.0)
            if j["type"] in ("revolute", "continuous"):
                Tj = Tj @ T_from(axis_angle_R(j["axis"], val), [0, 0, 0])
            elif j["type"] == "prismatic":
                ax = np.asarray(j["axis"], float)
                Tj = Tj @ T_from(np.eye(3), ax / np.linalg.norm(ax) * val)
            T[c] = Tj
            stack.append(c)
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--mesh-dir", required=True)
    ap.add_argument("--config", choices=["6dof", "5dof"], default="6dof")
    ap.add_argument("--roll-min", type=float, default=-90.0)
    ap.add_argument("--roll-max", type=float, default=90.0)
    ap.add_argument("--roll-step", type=float, default=5.0)
    ap.add_argument("--slab-x", default="0.225,0.375")
    ap.add_argument("--slab-z", default="0.0,0.4")
    ap.add_argument("--samples", type=int, default=300000, help="surface samples per link")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--compare", help="clearance_curve.py CSV with measured min_valid_gap_m")
    ap.add_argument("--out", default="mesh_width.csv")
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    joints, roots, bodies = parse(os.path.expanduser(a.urdf), a.mesh_dir, a.samples, rng)
    x0, x1 = [float(v) for v in a.slab_x.split(",")]
    z0, z1 = [float(v) for v in a.slab_z.split(",")]
    rolls = ([0.0] if a.config == "5dof"
             else [float(r) for r in np.arange(a.roll_min, a.roll_max + 1e-9, a.roll_step)])
    measured = {}
    if a.compare:
        for row in csv.DictReader(open(os.path.expanduser(a.compare))):
            if row["min_valid_gap_m"] != "":
                measured[float(row["roll_deg"])] = 1000.0 * float(row["min_valid_gap_m"])

    print(f"links with collision meshes: {sorted(bodies)}")
    print(f"slab x [{x0}, {x1}]  z [{z0}, {z1}]\n")
    print("roll   predicted_mm  limiting_link        2nd_link (mm)" + ("    measured_mm   diff_mm" if measured else ""))
    out = []
    for r in rolls:
        q = dict(BASE[a.config])
        if a.config == "6dof":
            q["joint5_roll"] = np.radians(r)
        T = link_poses(joints, roots, q)
        widths = {}
        for name, (pts, Tc, fn) in bodies.items():
            if name not in T:
                continue
            P = (T[name] @ Tc @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
            m = (P[:, 0] >= x0) & (P[:, 0] <= x1) & (P[:, 2] >= z0) & (P[:, 2] <= z1)
            if m.any():
                y = P[m, 1]
                widths[name] = 2.0 * max(y.max(), -y.min()) * 1000.0
        ranked = sorted(widths.items(), key=lambda kv: -kv[1])
        if not ranked:
            print(f"{r:6.1f}  no link inside the slab")
            continue
        lim, w = ranked[0]
        second = f"{ranked[1][0]} {ranked[1][1]:.1f}" if len(ranked) > 1 else ""
        line = f"{r:6.1f}   {w:10.1f}  {lim:<20} {second}"
        row = [a.config, r, round(w, 2), lim, second]
        if r in measured:
            line += f"   {measured[r]:10.1f}   {w - measured[r]:+7.1f}"
            row += [measured[r], round(w - measured[r], 2)]
        print(line)
        out.append(row)
    with open(os.path.expanduser(a.out), "w", newline="") as f:
        w_ = csv.writer(f)
        w_.writerow(["config", "roll_deg", "predicted_mm", "limiting_link", "second_link", "measured_mm", "diff_mm"])
        w_.writerows(out)
    print("\nwrote", a.out)


if __name__ == "__main__":
    main()
