#!/usr/bin/env python3
"""
Static clearance curve: for every tool roll, the smallest corridor gap at which the EXACT goal pose is collision-free.
No planning and no goal tolerance involved.

Needs a RUNNING move_group for the arm under test (its demo.launch.py), in a shell with /opt/ros/jazzy sourced
(no venv). It uses two services: /apply_planning_scene (places the walls) and /check_state_validity.

  python3 clearance_curve.py --config 6dof --out ~/analysis/clearance_6dof.csv
  python3 clearance_curve.py --config 5dof --out ~/analysis/clearance_5dof.csv

Scene: walls 0.15 x 0.14 x 0.40 at x = 0.30, y = +-wall_y, z = 0.2 (same as scene_builder_variable.py), plus the
bottom plate; the top barrier is removed. Clear gap = 2*wall_y - 0.14.
For each gap on the grid it checks every roll; a roll's answer is the smallest gap on the grid that is valid.
Rolls where validity is not monotone in the gap are flagged in the CSV (they should not occur).
Joints 1-4 are the shared roll-0 goal used throughout (6-DOF: final_goals.json; 5-DOF: goals_5dof_all.json goal 0).
The 5-DOF has no roll joint, so it is evaluated at roll 0 only.
NOT yet run against a live move_group: if a service call fails, paste the error.
"""
import argparse
import csv
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive

WALL_T = 0.14
BASE = {"6dof": [0.0, 0.09061371837047713, 0.19387853686569093, -0.28449225523616806],
        "5dof": [0.0, 0.34549575828472284, -0.1686695073991569, -0.1768262508855657]}
NAMES = {"6dof": ["joint1", "joint2", "joint3", "joint4", "joint5_roll"],
         "5dof": ["joint1", "joint2", "joint3", "joint4"]}


class Clearance(Node):
    def __init__(self, group):
        super().__init__("clearance_curve")
        self.group = group
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid = self.create_client(GetStateValidity, "/check_state_validity")
        for cli, name in ((self.scene, "/apply_planning_scene"), (self.valid, "/check_state_validity")):
            if not cli.wait_for_service(timeout_sec=15.0):
                sys.exit(f"{name} not available: is move_group running?")

    @staticmethod
    def box(name, dims, pos):
        o = CollisionObject()
        o.id, o.header.frame_id, o.operation = name, "world", CollisionObject.ADD
        p = SolidPrimitive()
        p.type, p.dimensions = SolidPrimitive.BOX, dims
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        pose.orientation.w = 1.0
        o.primitives.append(p)
        o.primitive_poses.append(pose)
        return o

    def apply(self, objs):
        sc = PlanningScene()
        sc.is_diff = True
        sc.world.collision_objects = objs
        req = ApplyPlanningScene.Request()
        req.scene = sc
        fut = self.scene.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result() is not None and fut.result().success

    def remove_barrier(self):
        o = CollisionObject()
        o.id, o.header.frame_id, o.operation = "top_barrier", "world", CollisionObject.REMOVE
        return self.apply([o])

    def set_gap(self, gap):
        wy = (gap + WALL_T) / 2.0
        ok = self.apply([self.box("left_wall", [0.15, WALL_T, 0.4], [0.30, -wy, 0.2]),
                         self.box("right_wall", [0.15, WALL_T, 0.4], [0.30, wy, 0.2]),
                         self.box("bottom_plate", [0.5, 0.5, 0.02], [0.30, 0.0, -0.01])])
        time.sleep(0.3)
        return ok

    def is_valid(self, names, values):
        req = GetStateValidity.Request()
        req.group_name = self.group
        rs = RobotState()
        rs.joint_state.name = list(names)
        rs.joint_state.position = [float(v) for v in values]
        req.robot_state = rs
        fut = self.valid.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.result() is None:
            sys.exit("check_state_validity call failed / timed out")
        return bool(fut.result().valid)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", choices=["6dof", "5dof"], required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--group", default="arm")
    ap.add_argument("--gap-min", type=float, default=0.040)
    ap.add_argument("--gap-max", type=float, default=0.150)
    ap.add_argument("--gap-step", type=float, default=0.001)
    ap.add_argument("--roll-min", type=float, default=-90.0)
    ap.add_argument("--roll-max", type=float, default=90.0)
    ap.add_argument("--roll-step", type=float, default=5.0)
    a = ap.parse_args()

    names, base = NAMES[a.config], BASE[a.config]
    rolls = ([0.0] if a.config == "5dof"
             else [float(r) for r in np.arange(a.roll_min, a.roll_max + 1e-9, a.roll_step)])
    gaps = [round(float(g), 4) for g in np.arange(a.gap_min, a.gap_max + 1e-9, a.gap_step)]

    rclpy.init()
    node = Clearance(a.group)
    print("removed top barrier (if present):", node.remove_barrier())

    grid = {r: [] for r in rolls}
    for i, g in enumerate(gaps):
        if not node.set_gap(g):
            sys.exit(f"could not apply scene at gap {g}")
        for r in rolls:
            vals = base + ([np.radians(r)] if a.config == "6dof" else [])
            grid[r].append(node.is_valid(names, vals))
        if i % 10 == 0:
            print(f"gap {g:.3f} done ({i + 1}/{len(gaps)})", flush=True)

    out = os.path.expanduser(a.out)
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    rows = []
    print("\nroll_deg   min_valid_gap_m   largest_invalid_gap_m   monotone   note")
    for r in rolls:
        v = grid[r]
        first = next((k for k, ok in enumerate(v) if ok), None)
        last_bad = max((k for k, ok in enumerate(v) if not ok), default=None)
        monotone = first is None or all(v[first:])
        note = ""
        if first is None:
            note = f"never valid up to {gaps[-1]}"
        elif first == 0:
            note = f"valid at the smallest gap tested ({gaps[0]}): lower bound not found"
        mg = gaps[first] if first is not None else ""
        lb = gaps[last_bad] if last_bad is not None else ""
        rows.append([a.config, r, mg, lb, int(monotone), note])
        print(f"{r:8.1f}   {mg!s:>15}   {lb!s:>21}   {int(monotone):>8}   {note}")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["config", "roll_deg", "min_valid_gap_m", "largest_invalid_gap_m", "monotone", "note"])
        w.writerows(rows)
    gridfile = out.replace(".csv", "_grid.csv")
    with open(gridfile, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gap_m"] + [f"roll_{r:g}" for r in rolls])
        for k, g in enumerate(gaps):
            w.writerow([g] + [int(grid[r][k]) for r in rolls])
    print(f"\nwrote {out} and {gridfile}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
