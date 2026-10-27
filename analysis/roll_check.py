#!/usr/bin/env python3
"""
Which tool rolls can this robot reach?  (forward kinematics only, no ROS)

Samples random joint values inside the URDF limits, computes the tool frame, and measures the tool "roll":
the rotation of the tool y-axis about its approach (x) axis, measured from the horizontal reference direction
(z_world x approach). Roll is folded modulo 180 deg, because a parallel gripper looks the same after a half turn.

  python3 roll_check.py robot_5dof.urdf --tip end_effector_link
  python3 roll_check.py robot_6dof.urdf --tip end_effector_link

Needs fk_check.py in the same folder. Prints, for each target roll, the smallest gap to any sampled tool roll.
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk_check import rpy_to_R, axis_angle_R, T_from, load  # noqa: E402


def limits(path):
    out = {}
    for j in ET.parse(path).getroot().findall("joint"):
        lim = j.find("limit")
        if lim is not None and lim.get("lower") is not None:
            out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--tip", default="end_effector_link")
    ap.add_argument("--samples", type=int, default=50000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--targets", default="0,15,30,45,60,75,90")
    ap.add_argument("--tol-deg", type=float, default=1.0)
    a = ap.parse_args()

    links, joints = load(a.urdf)
    lim = limits(a.urdf)
    if a.tip not in links:
        sys.exit(f"tip '{a.tip}' not in URDF")
    chain, link = [], a.tip
    while link in joints:
        chain.append(joints[link])
        link = joints[link]["parent"]
    chain.reverse()
    movable = [j for j in chain if j["type"] in ("revolute", "continuous")]
    print(f"{a.urdf}\nmovable joints in chain to {a.tip}: {[j['name'] for j in movable]}")

    rng = np.random.default_rng(a.seed)
    rolls = []
    for _ in range(a.samples):
        T = np.eye(4)
        for j in chain:
            T = T @ T_from(rpy_to_R(*j["rpy"]), j["xyz"])
            if j in movable:
                lo, hi = lim.get(j["name"], (-np.pi, np.pi))
                T = T @ T_from(axis_angle_R(j["axis"], rng.uniform(lo, hi)), [0, 0, 0])
        x, y = T[:3, 0], T[:3, 1]
        ref = np.cross([0, 0, 1.0], x)
        n = np.linalg.norm(ref)
        if n < 1e-3:          # approach axis vertical: roll reference undefined, skip
            continue
        ref /= n
        r = np.degrees(np.arctan2(np.dot(np.cross(ref, y), x), np.dot(ref, y)))
        rolls.append(((r + 90.0) % 180.0) - 90.0)   # fold to [-90, 90)
    rolls = np.abs(np.array(rolls))
    print(f"samples used: {len(rolls)}")
    print(f"tool roll (folded, abs) : min {rolls.min():.3f}  max {rolls.max():.3f} deg")
    print(f"fraction within {a.tol_deg:.1f} deg of zero roll: {np.mean(rolls < a.tol_deg):.3f}")
    print("\ntarget roll   smallest gap to any sampled tool roll   reachable?")
    for t in [float(v) for v in a.targets.split(",")]:
        gap = float(np.min(np.abs(rolls - t)))
        print(f"  {t:5.0f} deg        {gap:8.3f} deg                       {'yes' if gap < a.tol_deg else 'NO'}")


if __name__ == "__main__":
    main()