#!/usr/bin/env python3
"""
planning_benchmark_runner_v2.py  -- standalone patched copy of planning_benchmark_runner.py
(the original is NOT modified).

Same experiment as before (same start state, planner, 10 attempts, 10 s limit, +/-0.01 rad goal
constraints, velocity/accel scaling 0.5), but it records what the old runner threw away:
  * real MoveIt error code NAME (table copied from `ros2 interface show moveit_msgs/msg/MoveItErrorCodes`)
  * error_code.source and error_code.message
  * MoveIt's own planning_time (moveit_planning_time) next to the wall-clock time (planning_time)
  * path_length / num_waypoints from result.planned_trajectory
  * full start_state, planner_id, wall_y, clear_gap, scene-measured gap, run_id
  * timeouts / rejections / send failures are saved as rows (never silently dropped)
It refuses to write into a database that already contains rows, so runs can never be mixed again.
It reads the live planning scene and refuses to run if the wall positions do not match wall_y.

Parameters (-p name:=value):
  config_type        6dof | 5dof                       (default 6dof)
  goals_file         json with {"goals":[...]} or {"poses":[{"roll":..,"solution":[..]}]}
  trials_per_goal    int (default 10)
  db_path            NEW sqlite file (must not contain rows)
  planning_time      seconds (default 10.0)
  wall_y             wall centre used by scene_builder_variable; <=0 means "no corridor expected"
  run_id             free-text label stored in every row
  skip_placeholders  true: skip goals that are all-zero joints with roll>0 (5-DOF placeholders)
"""
import json
import math
import os
import sqlite3
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

WALL_THICKNESS = 0.14  # y-size of the wall boxes in scene_builder_variable.py

# From `ros2 interface show moveit_msgs/msg/MoveItErrorCodes` on this machine (Jazzy).
CODE_NAMES = {
    1: "SUCCESS", 0: "UNDEFINED", 99999: "FAILURE",
    -1: "PLANNING_FAILED", -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE", -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA", -6: "TIMED_OUT", -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION", -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION", -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED", -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS", -17: "INVALID_ROBOT_STATE", -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME", -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE", -23: "ROBOT_STATE_STALE", -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE", -26: "START_STATE_INVALID", -27: "GOAL_STATE_INVALID",
    -28: "UNRECOGNIZED_GOAL_TYPE", -29: "CRASH", -30: "ABORT", -31: "NO_IK_SOLUTION",
}
# Codes made up by THIS script for cases where MoveIt never returned a result.
CLIENT_SEND_FAILED = -9001
CLIENT_GOAL_HANDLE_REJECTED = -9002
CLIENT_RESULT_TIMEOUT = -9003
CODE_NAMES.update({
    CLIENT_SEND_FAILED: "CLIENT_SEND_FAILED",
    CLIENT_GOAL_HANDLE_REJECTED: "CLIENT_GOAL_HANDLE_REJECTED",
    CLIENT_RESULT_TIMEOUT: "CLIENT_RESULT_TIMEOUT",
})


def load_goals(path):
    d = json.load(open(os.path.expanduser(path)))
    goals, poses = d.get("goals") or [], d.get("poses") or []
    out = []
    if goals:
        for i, g in enumerate(goals):
            roll = poses[i].get("roll") if i < len(poses) else None
            out.append((i, roll, g))
    else:
        for i, p in enumerate(poses):
            out.append((i, p.get("roll"), p["solution"]))
    return out


class RunnerV2(Node):
    def __init__(self):
        super().__init__("planning_benchmark_runner_v2")
        self.declare_parameter("config_type", "6dof")
        self.declare_parameter("goals_file", "")
        self.declare_parameter("trials_per_goal", 10)
        self.declare_parameter("db_path", "")
        self.declare_parameter("planning_time", 10.0)
        self.declare_parameter("wall_y", -1.0)
        self.declare_parameter("run_id", "")
        self.declare_parameter("skip_placeholders", True)
        g = lambda n: self.get_parameter(n).value
        self.config_type = g("config_type")
        self.trials_per_goal = int(g("trials_per_goal"))
        self.db_path = os.path.expanduser(g("db_path"))
        self.planning_time = float(g("planning_time"))
        self.wall_y = float(g("wall_y"))
        self.run_id = g("run_id") or datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.skip_placeholders = bool(g("skip_placeholders"))

        if self.config_type not in ("6dof", "5dof"):
            sys.exit("config_type must be 6dof or 5dof")
        if not self.db_path:
            sys.exit("db_path is required")
        if not g("goals_file") or not os.path.exists(os.path.expanduser(g("goals_file"))):
            sys.exit(f"goals_file not found: {g('goals_file')}")

        self.joint_names = (["joint1", "joint2", "joint3", "joint4", "joint5_roll"]
                            if self.config_type == "6dof" else ["joint1", "joint2", "joint3", "joint4"])
        self.clear_gap = round(2 * self.wall_y - WALL_THICKNESS, 4) if self.wall_y > 0 else None

        # goals
        self.goals = []
        for gid, roll, joints in load_goals(g("goals_file")):
            vals = list(joints)[:len(self.joint_names)]
            placeholder = (all(abs(v) < 1e-9 for v in vals) and roll not in (None, 0))
            if placeholder and self.skip_placeholders:
                self.get_logger().warn(f"goal {gid} (roll {roll}) is an all-zero placeholder: SKIPPED")
                continue
            self.goals.append((gid, roll, vals))
        if not self.goals:
            sys.exit("no usable goals")

        self.client = ActionClient(self, MoveGroup, "/move_action")
        if not self.client.wait_for_server(timeout_sec=30.0):
            sys.exit("MoveGroup action server /move_action not available")
        self.scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.measured_gap = self.check_scene()
        self.init_database()   # only after the scene check passed; exits if db already has rows

    # ------------------------------------------------------------------ setup
    def init_database(self):
        os.makedirs(os.path.dirname(self.db_path) or ".", exist_ok=True)
        con = sqlite3.connect(self.db_path)
        con.execute("""CREATE TABLE IF NOT EXISTS planning_results (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            run_id TEXT, timestamp TEXT, config_type TEXT,
            goal_id INTEGER, goal_roll_deg REAL, trial_number INTEGER, global_trial INTEGER,
            success INTEGER,
            planning_time REAL,            -- wall clock, comparable with the old databases
            moveit_planning_time REAL,     -- reported by MoveIt in the result
            error_code INTEGER, error_message TEXT,   -- error_message = NAME of the code
            error_source TEXT, error_detail TEXT,
            planner_id TEXT, num_planning_attempts INTEGER,
            goal_config TEXT, path_length REAL, num_waypoints INTEGER,
            start_state TEXT, planning_timeout REAL,
            wall_y REAL, clear_gap REAL, measured_gap REAL)""")
        n = con.execute("SELECT COUNT(*) FROM planning_results").fetchone()[0]
        con.commit()
        con.close()
        if n > 0:
            sys.exit(f"REFUSING: {self.db_path} already has {n} rows. Use a new db_path.")

    def check_scene(self):
        """Read the live planning scene; verify wall positions match wall_y. Returns measured gap or None."""
        if not self.scene_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("/get_planning_scene not available: scene NOT verified")
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        fut = self.scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            self.get_logger().warn("get_planning_scene failed: scene NOT verified")
            return None
        objs = {o.id: o for o in res.scene.world.collision_objects}
        self.get_logger().info(f"scene objects: {sorted(objs)}")
        if self.wall_y <= 0:
            if "left_wall" in objs or "right_wall" in objs:
                sys.exit("wall_y<=0 (no corridor expected) but walls are in the scene")
            return None
        if "left_wall" not in objs or "right_wall" not in objs:
            sys.exit("walls missing from planning scene: run scene_builder_variable first")
        lo, ro = objs["left_wall"], objs["right_wall"]
        self.get_logger().info(
            f"left_wall obj.pose.y={lo.pose.position.y:.3f} shape.y={lo.primitive_poses[0].position.y:.3f} | "
            f"right_wall obj.pose.y={ro.pose.position.y:.3f} shape.y={ro.primitive_poses[0].position.y:.3f}")
        # MoveIt returns shape poses relative to the object pose; orientation is identity here, so add them
        ly = lo.pose.position.y + lo.primitive_poses[0].position.y
        ry = ro.pose.position.y + ro.primitive_poses[0].position.y
        thick = objs["left_wall"].primitives[0].dimensions[1]
        gap = round((ry - ly) - thick, 4)
        self.get_logger().info(f"scene walls at y={ly:.3f}/{ry:.3f}, thickness {thick:.3f} -> clear gap {gap:.3f} m")
        if abs(gap - self.clear_gap) > 1e-3:
            sys.exit(f"SCENE MISMATCH: scene gap {gap} m but wall_y={self.wall_y} implies {self.clear_gap} m")
        return gap

    # --------------------------------------------------------------- request
    def start_state(self):
        # identical to the original runner (joint2 = -37 deg, gripper joints not set)
        s = {"joint1": 0.0, "joint2": -0.6458, "joint3": 0.0, "joint4": 0.0}
        if self.config_type == "6dof":
            s["joint5_roll"] = 0.0
        return s

    def make_request(self, joints, start):
        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = "world"
        for ax in "xyz":
            setattr(req.workspace_parameters.min_corner, ax, -1.0)
            setattr(req.workspace_parameters.max_corner, ax, 1.0)
        req.group_name = "arm"
        req.num_planning_attempts = 10
        req.allowed_planning_time = self.planning_time
        req.planner_id = "RRTConnectkConfigDefault"
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        c = Constraints()
        for name, val in zip(self.joint_names, joints):
            jc = JointConstraint()
            jc.joint_name, jc.position = name, float(val)
            jc.tolerance_above = jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        req.start_state.joint_state.name = list(start.keys())
        req.start_state.joint_state.position = list(start.values())
        req.start_state.joint_state.velocity = [0.0] * len(start)
        return req

    @staticmethod
    def path_length(traj):
        pts = traj.joint_trajectory.points
        if len(pts) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(pts[:-1], pts[1:]):
            total += float(np.linalg.norm(np.array(b.positions) - np.array(a.positions)))
        return total

    # ----------------------------------------------------------------- trial
    def run_trial(self, gid, roll, joints, trial, global_trial):
        start = self.start_state()
        goal = MoveGroup.Goal()
        goal.request = self.make_request(joints, start)
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0

        rec = dict(success=0, moveit_time=None, code=None, source="", detail="",
                   plen=0.0, nwp=0)
        t0 = time.time()
        sf = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=self.planning_time + 5.0)
        handle = sf.result()
        if handle is None:
            rec["code"] = CLIENT_SEND_FAILED
        elif not handle.accepted:
            rec["code"] = CLIENT_GOAL_HANDLE_REJECTED
        else:
            rf = handle.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=self.planning_time + 5.0)
            wrapped = rf.result()
            if wrapped is None:
                rec["code"] = CLIENT_RESULT_TIMEOUT
            else:
                r = wrapped.result
                rec["code"] = int(r.error_code.val)
                rec["source"] = r.error_code.source
                rec["detail"] = r.error_code.message
                rec["moveit_time"] = float(r.planning_time)
                if rec["code"] == 1:
                    rec["success"] = 1
                    rec["plen"] = self.path_length(r.planned_trajectory)
                    rec["nwp"] = len(r.planned_trajectory.joint_trajectory.points)
        wall = time.time() - t0
        name = CODE_NAMES.get(rec["code"], f"UNKNOWN_{rec['code']}")

        con = sqlite3.connect(self.db_path)
        con.execute("""INSERT INTO planning_results
            (run_id,timestamp,config_type,goal_id,goal_roll_deg,trial_number,global_trial,success,
             planning_time,moveit_planning_time,error_code,error_message,error_source,error_detail,
             planner_id,num_planning_attempts,goal_config,path_length,num_waypoints,start_state,
             planning_timeout,wall_y,clear_gap,measured_gap)
            VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
            (self.run_id, datetime.now().isoformat(), self.config_type, gid, roll, trial, global_trial,
             rec["success"], wall, rec["moveit_time"], rec["code"], name, rec["source"], rec["detail"],
             goal.request.planner_id, goal.request.num_planning_attempts, json.dumps(joints),
             rec["plen"], rec["nwp"], json.dumps(start), self.planning_time,
             self.wall_y if self.wall_y > 0 else None, self.clear_gap, self.measured_gap))
        con.commit()
        con.close()
        self.get_logger().info(
            f"goal {gid} roll {roll} trial {trial:2d}: {'OK ' if rec['success'] else 'FAIL'} "
            f"{wall:5.2f}s {name} {rec['source']} {rec['detail']}")
        return rec["success"], name

    def run(self):
        n_goals = len(self.goals)
        self.get_logger().info(
            f"RUN {self.run_id} | {self.config_type} | goals {n_goals} x {self.trials_per_goal} "
            f"| wall_y {self.wall_y} gap {self.clear_gap} measured {self.measured_gap} | db {self.db_path}")
        total, ok, gt = 0, 0, 0
        causes = {}
        per_goal = []
        for gid, roll, joints in self.goals:
            gok = 0
            for t in range(1, self.trials_per_goal + 1):
                gt += 1
                s, name = self.run_trial(gid, roll, joints, t, gt)
                gok += s
                total += 1
                if not s:
                    causes[name] = causes.get(name, 0) + 1
                time.sleep(0.5)
            ok += gok
            per_goal.append((gid, roll, gok))
        self.get_logger().info("=" * 60)
        for gid, roll, gok in per_goal:
            self.get_logger().info(f"goal {gid} (roll {roll}): {gok}/{self.trials_per_goal}")
        self.get_logger().info(f"TOTAL {ok}/{total}  failure causes: {causes}")


def main():
    rclpy.init(args=sys.argv)
    node = RunnerV2()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()