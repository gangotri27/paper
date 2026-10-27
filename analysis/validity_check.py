#!/usr/bin/env python3
"""
Asks a RUNNING move_group whether the benchmark's start and goal states are collision-free, using the
/check_state_validity service. Nothing is planned or moved.

It answers three questions for each goal in a goals file:
  1. Is the exact goal state valid?
  2. If MoveIt samples a state anywhere inside the +/- tol box the runner uses (0.01 rad), what fraction is valid?
  3. When a state is invalid, which pairs of bodies are in contact?

Run it with the corridor present in the planning scene (and again without it, as a control).

  python3 validity_check.py --config 6dof --goals-file ~/new_ws_review/.../final_goals.json
  python3 validity_check.py --config 5dof --goals-file ~/benchmark_test_original/goals_5dof_single.json

Goals files seen so far store either {"goals": [...]} or {"poses": [{"roll":..,"solution":[...]}]}; both are handled.
NOT yet run against a live move_group: if the service call fails, paste the error and we will adjust.
"""
import argparse
import json
import os
import random
import sys
from collections import Counter

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

JOINTS = {"6dof": ["joint1", "joint2", "joint3", "joint4", "joint5_roll"],
          "5dof": ["joint1", "joint2", "joint3", "joint4"]}
# same start state the benchmark runner used (planning_benchmark_runner.get_start_state)
START = {"joint1": 0.0, "joint2": -0.6458, "joint3": 0.0, "joint4": 0.0, "joint5_roll": 0.0}


class Checker(Node):
    def __init__(self, group):
        super().__init__("validity_check")
        self.group = group
        self.cli = self.create_client(GetStateValidity, "/check_state_validity")
        if not self.cli.wait_for_service(timeout_sec=15.0):
            sys.exit("/check_state_validity not available: is move_group running?")

    def check(self, names, values):
        req = GetStateValidity.Request()
        req.group_name = self.group
        rs = RobotState()
        rs.joint_state.name = list(names)
        rs.joint_state.position = [float(v) for v in values]
        req.robot_state = rs
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            sys.exit("service call failed / timed out")
        pairs = [tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts]
        return bool(res.valid), pairs


def load_goals(path):
    d = json.load(open(os.path.expanduser(path)))
    poses = d.get("poses") or []
    goals = d.get("goals") or []
    out = []
    if goals:
        for i, g in enumerate(goals):
            roll = poses[i].get("roll") if i < len(poses) else None
            out.append((i, roll, g))
    else:
        for i, p in enumerate(poses):
            out.append((i, p.get("roll"), p["solution"]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", choices=["6dof", "5dof"], required=True)
    ap.add_argument("--goals-file", required=True)
    ap.add_argument("--group", default="arm")
    ap.add_argument("--samples", type=int, default=200)
    ap.add_argument("--tol", type=float, default=0.01)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    random.seed(a.seed)
    names = JOINTS[a.config]
    rclpy.init()
    node = Checker(a.group)

    ok, pairs = node.check(names, [START[n] for n in names])
    print(f"START state valid: {ok}   contacts: {Counter(pairs).most_common(5)}")

    print(f"\n{'goal':>4} {'roll':>5} {'exact':>6} {'valid in +/-%.3f box' % a.tol:>22}   top contact pairs when invalid" )
    for gid, roll, g in load_goals(a.goals_file):
        vals = g[:len(names)]
        exact, p0 = node.check(names, vals)
        nvalid, allpairs = 0, []
        for _ in range(a.samples):
            s = [v + random.uniform(-a.tol, a.tol) for v in vals]
            v, p = node.check(names, s)
            nvalid += v
            allpairs += p
        allpairs += p0
        top = Counter(allpairs).most_common(2)
        print(f"{gid:>4} {str(roll):>5} {str(exact):>6} {nvalid:>10}/{a.samples:<10}   {top}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
