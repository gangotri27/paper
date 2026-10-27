#!/usr/bin/env python3
"""
Forward kinematics straight from a URDF (numpy + xml only, no ROS needed).

Examples
  python3 fk_check.py robot.urdf --list-leaves
  python3 fk_check.py robot.urdf --list-joints
  python3 fk_check.py robot.urdf --tip end_effector_link --q joint1=0 joint2=0.0906 joint3=0.1939 joint4=-0.2845 joint5_roll=0

Prints the tip-link position (m) and orientation in the URDF's root frame, so you can compare two robots.
Joints not given with --q are taken as 0 and listed as a warning.
"""
import argparse
import sys
import xml.etree.ElementTree as ET
import numpy as np


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF convention (fixed-axis rpy)


def axis_angle_R(axis, a):
    k = np.asarray(axis, float)
    n = np.linalg.norm(k)
    k = k / n if n > 0 else np.array([1.0, 0, 0])
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)


def T_from(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def floats(s, default):
    return [float(x) for x in s.split()] if s else default


def load(urdf_path):
    root = ET.parse(urdf_path).getroot()
    links = [l.get("name") for l in root.findall("link")]
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        ax = j.find("axis")
        joints[j.find("child").get("link")] = dict(
            name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
            xyz=floats(o.get("xyz") if o is not None else None, [0, 0, 0]),
            rpy=floats(o.get("rpy") if o is not None else None, [0, 0, 0]),
            axis=floats(ax.get("xyz") if ax is not None else None, [1, 0, 0]))
    return links, joints


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--tip")
    ap.add_argument("--q", nargs="*", default=[], help="joint=value (rad), e.g. joint1=0.3")
    ap.add_argument("--list-leaves", action="store_true")
    ap.add_argument("--list-joints", action="store_true")
    a = ap.parse_args()

    links, joints = load(a.urdf)
    children = set(joints)
    parents = {j["parent"] for j in joints.values()}
    roots = [l for l in links if l not in children]

    if a.list_leaves:
        print("root link(s):", roots)
        print("leaf links (candidate tips):", [l for l in links if l not in parents])
        return
    if a.list_joints:
        for child, j in joints.items():
            print(f"{j['name']:28s} {j['type']:10s} {j['parent']} -> {child}")
        return
    if not a.tip:
        sys.exit("give --tip (use --list-leaves to see candidates)")
    if a.tip not in links:
        sys.exit(f"tip link '{a.tip}' not in URDF. Candidates: {[l for l in links if l not in parents]}")

    q = {}
    for item in a.q:
        k, v = item.split("=")
        q[k] = float(v)

    chain, link = [], a.tip
    while link in joints:
        chain.append(joints[link])
        link = joints[link]["parent"]
    chain.reverse()
    base = link

    T = np.eye(4)
    missing = []
    used = set()
    for j in chain:
        T = T @ T_from(rpy_to_R(*j["rpy"]), j["xyz"])
        if j["type"] in ("revolute", "continuous"):
            val = q.get(j["name"])
            if val is None:
                missing.append(j["name"])
                val = 0.0
            used.add(j["name"])
            T = T @ T_from(axis_angle_R(j["axis"], val), [0, 0, 0])
        elif j["type"] == "prismatic":
            val = q.get(j["name"])
            if val is None:
                missing.append(j["name"])
                val = 0.0
            used.add(j["name"])
            T = T @ T_from(np.eye(3), np.asarray(j["axis"], float) / np.linalg.norm(j["axis"]) * val)

    R, p = T[:3, :3], T[:3, 3]
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    print(f"URDF: {a.urdf}")
    print(f"frame: {base} -> {a.tip}   ({len(chain)} joints in chain)")
    print("chain:", " , ".join(f"{j['name']}[{j['type']}]" for j in chain))
    print(f"position (m):   x={p[0]:.4f}  y={p[1]:.4f}  z={p[2]:.4f}")
    print(f"orientation rpy (rad): roll={roll:.4f} pitch={pitch:.4f} yaw={yaw:.4f}   (deg: {np.degrees(roll):.1f}, {np.degrees(pitch):.1f}, {np.degrees(yaw):.1f})")
    print(f"tip x-axis (approach) in {base}: [{R[0,0]:.3f}, {R[1,0]:.3f}, {R[2,0]:.3f}]")
    unknown = [k for k in q if k not in used]
    if missing:
        print("WARNING: joints in chain with no value given (used 0):", missing)
    if unknown:
        print("WARNING: values given for joints NOT in this chain (ignored):", unknown)


if __name__ == "__main__":
    main()
