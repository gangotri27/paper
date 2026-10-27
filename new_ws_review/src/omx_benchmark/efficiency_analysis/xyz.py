#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point
import pandas as pd
import numpy as np
import argparse
import math
import time
import json
from datetime import datetime, timezone
import random


class EfficiencyBenchmark(Node):
    def __init__(self, args):
        super().__init__("efficiency_benchmark")
        self.args = args
        random.seed(args.seed)
        np.random.seed(args.seed)

        self.client = self.create_client(GetMotionPlan, "/plan_kinematic_path")

        self.configs = {
            "6dof": {
                "group_name": "arm",
                "joints": ["joint1", "joint2", "joint3", "joint4", "joint5_roll"],
                "limits": [
                    (-math.pi, math.pi),
                    (-1.5, 1.5),
                    (-1.5, 1.4),
                    (-1.7, 1.97),
                    (-math.pi, math.pi),
                ],
                "end_link": "end_effector_link",
            },
            "5dof": {
                "group_name": "arm",
                "joints": ["joint1", "joint2", "joint3", "joint4"],
                "limits": [
                    (-math.pi, math.pi),
                    (-1.5, 1.5),
                    (-1.5, 1.4),
                    (-1.7, 1.97),
                ],
                "end_link": "end_effector_link",
            },
        }

    def wait_for_services(self):
        self.get_logger().info("Waiting for MoveIt /plan_kinematic_path service...")
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("MoveIt planning service not available!")
            return False
        return True

    def compute_joint_path_length(self, trajectory):
        length = 0.0
        points = trajectory.joint_trajectory.points
        for i in range(1, len(points)):
            prev = np.array(points[i - 1].positions)
            curr = np.array(points[i].positions)
            length += np.linalg.norm(curr - prev)
        return float(length)

    def compute_duration(self, trajectory):
        points = trajectory.joint_trajectory.points
        if not points:
            return 0.0
        last_pt = points[-1]
        return float(
            last_pt.time_from_start.sec
            + last_pt.time_from_start.nanosec * 1e-9
        )

    def generate_random_state(self, limits):
        return [random.uniform(lim[0], lim[1]) for lim in limits]

    # ✅ POSITION-ONLY CONSTRAINT FUNCTION
    def create_target_constraints(self, config_name, x, y, z):
        config = self.configs[config_name]
        goal_constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = self.args.frame_id
        pc.link_name = config["end_link"]

        bv = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [self.args.pos_tol]  # sphere tolerance
        bv.primitives.append(prim)

        pose = Pose()
        pose.position = Point(x=float(x), y=float(y), z=float(z))
        bv.primitive_poses.append(pose)

        pc.constraint_region = bv
        pc.weight = 1.0
        goal_constraints.position_constraints.append(pc)

        return [goal_constraints]

    def run_benchmark(self, config_name):
        if not self.wait_for_services():
            return

        print(f"\n🚀 STARTING BENCHMARK: {config_name.upper()} ARM")
        print("-" * 80)

        config = self.configs[config_name]
        num_joints = len(config["joints"])

        try:
            df = pd.read_csv("xyz.csv")
        except FileNotFoundError:
            self.get_logger().error("xyz.csv not found!")
            return

        results = []
        joint_log = []
        num_trials = min(self.args.trials, len(df))

        if num_trials == 0:
            print("❌ No trials to run (CSV might be empty).")
            return

        current_start_state = [0.0] * num_joints

        for trial_idx in range(num_trials):
            row = df.iloc[trial_idx]
            x, y, z = row["x"], row["y"], row["z"]

            # Start mode selection
            if self.args.start_mode == "home":
                current_start_state = [0.0] * num_joints
            elif self.args.start_mode == "random":
                current_start_state = self.generate_random_state(
                    config["limits"]
                )

            start_state_used = list(current_start_state)

            req = GetMotionPlan.Request()
            req.motion_plan_request.group_name = config["group_name"]
            req.motion_plan_request.num_planning_attempts = (
                self.args.num_planning_attempts
            )
            req.motion_plan_request.allowed_planning_time = (
                self.args.planning_time
            )

            req.motion_plan_request.start_state.joint_state.header.stamp = (
                self.get_clock().now().to_msg()
            )
            req.motion_plan_request.start_state.joint_state.name = (
                config["joints"]
            )
            req.motion_plan_request.start_state.joint_state.position = (
                start_state_used
            )

            req.motion_plan_request.goal_constraints = (
                self.create_target_constraints(config_name, x, y, z)
            )

            # Call planner
            start_wall = time.time()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            end_wall = time.time()

            success = False
            planning_time = 0.0
            path_length = 0.0
            traj_duration = 0.0
            final_joints = []
            waypoints = []

            if future.result() is not None:
                resp = future.result().motion_plan_response

                if (
                    resp.error_code.val
                    == resp.error_code.SUCCESS
                    and len(resp.trajectory.joint_trajectory.points) > 0
                ):
                    success = True
                    planning_time = resp.planning_time
                    trajectory = resp.trajectory

                    path_length = self.compute_joint_path_length(
                        trajectory
                    )
                    traj_duration = self.compute_duration(trajectory)

                    for pt in trajectory.joint_trajectory.points:
                        waypoints.append(
                            [float(pos) for pos in pt.positions]
                        )

                    final_joints = [
                        float(pos)
                        for pos in trajectory.joint_trajectory.points[
                            -1
                        ].positions
                    ]

                    if self.args.start_mode == "chain":
                        current_start_state = final_joints

            wall_time = end_wall - start_wall

            results.append(
                {
                    "trial": trial_idx,
                    "config": config_name,
                    "success": success,
                    "planning_time_sec": planning_time,
                    "wall_time_sec": wall_time,
                    "path_length_rad": path_length,
                    "trajectory_duration_sec": traj_duration,
                    "timestamp_utc": datetime.now(
                        timezone.utc
                    ).isoformat(),
                    "start_state_json": json.dumps(
                        start_state_used
                    ),
                    "goal_xyz_json": json.dumps([x, y, z]),
                    "waypoints_json": json.dumps(waypoints),
                }
            )

            joint_log.append(
                {
                    "trial": trial_idx,
                    "config": config_name,
                    "success": success,
                    "x": x,
                    "y": y,
                    "z": z,
                    "final_joints_json": json.dumps(final_joints)
                    if success
                    else "[]",
                }
            )

            status = "✅ SUCCESS" if success else "❌ FAILED"
            print(
                f"[{config_name.upper()}] Trial {trial_idx+1}/{num_trials} "
                f"-> {status} | Plan Time: {planning_time:.3f}s"
            )

        res_file = f"benchmark_results_{config_name}.csv"
        jnt_file = f"benchmark_joint_log_{config_name}.csv"

        pd.DataFrame(results).to_csv(res_file, index=False)
        pd.DataFrame(joint_log).to_csv(jnt_file, index=False)

        print(f"\n💾 Saved Metrics to: {res_file}")
        print(f"💾 Saved Joint Data to: {jnt_file}")

        success_rate = (
            sum(1 for r in results if r["success"])
            / num_trials
            * 100
            if num_trials > 0
            else 0.0
        )

        print(
            f"📊 {config_name.upper()} Overall Success Rate: "
            f"{success_rate:.1f}%\n"
        )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", required=True, choices=["5dof", "6dof", "both"]
    )
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--planning_time", type=float, default=2.0)
    parser.add_argument("--pos_tol", type=float, default=0.005)
    parser.add_argument("--num_planning_attempts", type=int, default=3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--frame_id",
        type=str,
        default="link1",
        help="Must match MoveIt planning frame",
    )
    parser.add_argument(
        "--start_mode",
        type=str,
        choices=["home", "random", "chain"],
        default="chain",
    )

    parsed_args, _ = parser.parse_known_args()

    rclpy.init()
    benchmark_node = EfficiencyBenchmark(parsed_args)

    if parsed_args.config == "both":
        benchmark_node.run_benchmark("5dof")
        benchmark_node.run_benchmark("6dof")
    else:
        benchmark_node.run_benchmark(parsed_args.config)

    benchmark_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()





# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from moveit_commander import MoveGroupCommander, RobotCommander
# import pandas as pd
# import numpy as np
# import argparse
# import math
# import time
# import json
# from datetime import datetime, timezone
# import random


# class EfficiencyBenchmark(Node):

#     def __init__(self, args):
#         super().__init__("efficiency_benchmark")

#         random.seed(args.seed)
#         np.random.seed(args.seed)

#         self.robot = RobotCommander()

#         self.args = args

#         self.configs = {
#             "6dof": {
#                 "joints": ["joint1", "joint2", "joint3", "joint4", "joint5_roll"],
#                 "end_link": "end_effector_link",
#                 "limits": [(-math.pi, math.pi), (-1.5, 1.5),
#                            (-1.5, 1.4), (-1.7, 1.97),
#                            (-math.pi, math.pi)]
#             },
#             "5dof": {
#                 "joints": ["joint1", "joint2", "joint3", "joint4"],
#                 "end_link": "end_effector_link",
#                 "limits": [(-math.pi, math.pi), (-1.5, 1.5),
#                            (-1.5, 1.4), (-1.7, 1.97)]
#             }
#         }

#     # --------------------------
#     # PATH LENGTH (Joint Space)
#     # --------------------------
#     def compute_joint_path_length(self, trajectory):
#         length = 0.0
#         points = trajectory.joint_trajectory.points

#         for i in range(1, len(points)):
#             prev = np.array(points[i - 1].positions)
#             curr = np.array(points[i].positions)
#             length += np.linalg.norm(curr - prev)

#         return float(length)

#     # --------------------------
#     # TRAJECTORY DURATION
#     # --------------------------
#     def compute_duration(self, trajectory):
#         points = trajectory.joint_trajectory.points
#         if not points:
#             return 0.0
#         return float(points[-1].time_from_start.sec +
#                      points[-1].time_from_start.nanosec * 1e-9)

#     # --------------------------
#     # RUN BENCHMARK
#     # --------------------------
#     def run(self, config_name):

#         config = self.configs[config_name]
#         group = MoveGroupCommander("arm")

#         group.set_planning_time(self.args.planning_time)
#         group.set_goal_position_tolerance(self.args.pos_tol)
#         group.set_num_planning_attempts(self.args.num_planning_attempts)

#         df = pd.read_csv("robot_efficiency_benchmark_100.csv")

#         results = []
#         joint_log = []

#         for trial_idx in range(min(self.args.trials, len(df))):

#             pose_xyz = df.iloc[trial_idx][["x", "y", "z"]].values.tolist()

#             if self.args.start_mode == "random":
#                 rand_state = []
#                 for lim in config["limits"]:
#                     rand_state.append(random.uniform(lim[0], lim[1]))
#                 group.set_joint_value_target(rand_state)
#                 group.go(wait=True)
#             else:
#                 group.set_named_target("home")
#                 group.go(wait=True)

#             start_state = group.get_current_joint_values()

#             group.set_position_target(pose_xyz, config["end_link"])

#             start_wall = time.time()
#             plan = group.plan()
#             end_wall = time.time()

#             success = False
#             planning_time = 0.0
#             path_length = 0.0
#             trajectory_duration = 0.0
#             waypoints = []
#             joint_angles = []

#             if plan and len(plan.joint_trajectory.points) > 0:
#                 success = True

#                 planning_time = end_wall - start_wall
#                 wall_time = planning_time

#                 path_length = self.compute_joint_path_length(plan)
#                 trajectory_duration = self.compute_duration(plan)

#                 for pt in plan.joint_trajectory.points:
#                     waypoints.append(list(pt.positions))

#                 joint_angles = list(plan.joint_trajectory.points[-1].positions)

#             elapsed_time = end_wall - start_wall

#             result = {
#                 "trial": trial_idx,
#                 "config": config_name,
#                 "success": success,
#                 "planning_time": planning_time,
#                 "wall_time": wall_time,
#                 "path_length_joint": path_length,
#                 "trajectory_duration": trajectory_duration,
#                 "timestamp_utc": datetime.now(timezone.utc).isoformat(),
#                 "elapsed_time_sec": elapsed_time,
#                 "start_state_json": json.dumps(start_state),
#                 "goal_pose_json": json.dumps(pose_xyz),
#                 "waypoints_json": json.dumps(waypoints)
#             }

#             joint_result = {
#                 "trial": trial_idx,
#                 "config": config_name,
#                 "x": pose_xyz[0],
#                 "y": pose_xyz[1],
#                 "z": pose_xyz[2],
#                 "joint_angles_json": json.dumps(joint_angles),
#                 "success": success
#             }

#             results.append(result)
#             joint_log.append(joint_result)

#             print(f"[{config_name}] Trial {trial_idx} → Success: {success}")

#         pd.DataFrame(results).to_csv(
#             f"benchmark_results_{config_name}.csv", index=False)

#         pd.DataFrame(joint_log).to_csv(
#             f"benchmark_joint_log_{config_name}.csv", index=False)


# # --------------------------
# # MAIN
# # --------------------------
# def main():

#     parser = argparse.ArgumentParser()

#     parser.add_argument("--config", required=True,
#                         choices=["5dof", "6dof", "both"])
#     parser.add_argument("--trials", type=int, default=100)
#     parser.add_argument("--planning_time", type=float, default=10.0)
#     parser.add_argument("--pos_tol", type=float, default=0.003)
#     parser.add_argument("--ori_tol", type=float, default=0.05)
#     parser.add_argument("--start_mode",
#                         choices=["home", "random"], default="home")
#     parser.add_argument("--seed", type=int, default=42)
#     parser.add_argument("--num_planning_attempts", type=int, default=2)

#     args = parser.parse_args()

#     rclpy.init()
#     node = EfficiencyBenchmark(args)

#     if args.config == "both":
#         node.run("5dof")
#         node.run("6dof")
#     else:
#         node.run(args.config)

#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()