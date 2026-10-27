#!/usr/bin/env python3
"""
Planning-Only Benchmark Runner
Tests planning success without execution or task validation
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
import sqlite3
import time
import numpy as np
from datetime import datetime
import json
import os
import random

class PlanningBenchmarkRunner(Node):
    def __init__(self):
        super().__init__('planning_benchmark_runner')
        
        # Parameters
        self.declare_parameter('config_type', '6dof')
        self.declare_parameter('goals_file', '')
        self.declare_parameter('trials_per_goal', 10)
        self.declare_parameter('db_path', '/tmp/planning_benchmark.db')
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('seed', 42)
        
        self.config_type = self.get_parameter('config_type').value
        self.goals_file = self.get_parameter('goals_file').value
        self.trials_per_goal = self.get_parameter('trials_per_goal').value
        self.db_path = self.get_parameter('db_path').value
        self.planning_time = self.get_parameter('planning_time').value
        self.seed = self.get_parameter('seed').value
        
        random.seed(self.seed)
        np.random.seed(self.seed)
        
        # Load goals
        if not os.path.exists(self.goals_file):
            raise FileNotFoundError(f'Goals file not found: {self.goals_file}')
        
        with open(self.goals_file, 'r') as f:
            goals_data = json.load(f)
            self.goals = goals_data['goals']
        
        self.get_logger().info(f'Loaded {len(self.goals)} goals for {self.config_type}')
        
        # Action client
        self.move_group_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        if not self.move_group_action_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('MoveGroup action server not available')
        
        self.get_logger().info('Connected to MoveGroup action server')
        self.verify_start_state()
        
        self.init_database()
    
    def init_database(self):
        """Create database for planning-only results"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS planning_results (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                config_type TEXT,
                goal_id INTEGER,
                trial_number INTEGER,
                
                -- Planning metrics
                success INTEGER,
                planning_time REAL,
                error_code INTEGER,
                error_message TEXT,
                planner_id TEXT,
                num_planning_attempts INTEGER,
                
                -- Goal configuration
                goal_config TEXT,
                
                -- Path metrics (if successful)
                path_length REAL,
                num_waypoints INTEGER,
                
                -- Metadata
                start_state TEXT,
                random_seed INTEGER,
                planning_timeout REAL
            )
        ''')
        
        # Create index for faster queries
        cursor.execute('''
            CREATE INDEX IF NOT EXISTS idx_config_goal 
            ON planning_results(config_type, goal_id)
        ''')
        
        conn.commit()
        conn.close()
        
        self.get_logger().info(f'Database initialized: {self.db_path}')
    
    def create_planning_request(self, goal_config, start_state):
        """Create motion planning request - handles both dict and list goals"""
        req = MotionPlanRequest()
        
        # Workspace parameters
        req.workspace_parameters.header.frame_id = "world"
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        
        req.group_name = 'arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = self.planning_time
        req.planner_id = "RRTConnectkConfigDefault"
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        
        # Goal constraints - handle both dict and list formats
        constraints = Constraints()
        
        # Define joint names in order
        if self.config_type == '6dof':
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll']
        else:
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Check if goal_config is a dict or list
        if isinstance(goal_config, dict):
            # Dict format: {joint_name: value}
            for joint_name in joint_names:
                if joint_name in goal_config:
                    joint_con = JointConstraint()
                    joint_con.joint_name = joint_name
                    joint_con.position = goal_config[joint_name]
                    joint_con.tolerance_above = 0.01
                    joint_con.tolerance_below = 0.01
                    joint_con.weight = 1.0
                    constraints.joint_constraints.append(joint_con)
        else:
            # List format: assume values in joint_names order
            for i, joint_name in enumerate(joint_names):
                if i < len(goal_config):
                    joint_con = JointConstraint()
                    joint_con.joint_name = joint_name
                    joint_con.position = goal_config[i]
                    joint_con.tolerance_above = 0.01
                    joint_con.tolerance_below = 0.01
                    joint_con.weight = 1.0
                    constraints.joint_constraints.append(joint_con)
        
        req.goal_constraints = [constraints]
        
        # Start state (already a dict)
        req.start_state.joint_state.name = list(start_state.keys())
        req.start_state.joint_state.position = list(start_state.values())
        req.start_state.joint_state.velocity = [0.0] * len(start_state)
        
        return req
    
    def get_start_state(self):
        """
        Get collision-free start state - SAME FOR BOTH ROBOTS in shared joints
        Using your discovered -37° pose for joint2
        """
        # COMMON start state for both robots (joints 1-4)
        common_joints = {
            'joint1': 0.0,
            'joint2': -0.6458,  # -37 degrees in radians (your discovered safe pose)
            'joint3': 0.0,
            'joint4': 0.0
        }
        
        if self.config_type == '6dof':
            # For 6-DOF, add joint5_roll but keep it NEUTRAL (0.0)
            # This ensures no unfair advantage at start
            return {
                'joint1': common_joints['joint1'],
                'joint2': common_joints['joint2'],
                'joint3': common_joints['joint3'],
                'joint4': common_joints['joint4'],
                'joint5_roll': 0.0  # NEUTRAL - not helping!
            }
        else:  # 5dof
            return common_joints.copy()
        # if self.config_type == '6dof':
        #     return {
        #         'joint1': 0.0,
        #         'joint2': 0.0,
        #         'joint3': 0.0,
        #         'joint4': 0.0,
        #         'joint5_roll': 0.0
        #     }
        # else:
        #     return {
        #         'joint1': 0.0,
        #         'joint2': 0.0,
        #         'joint3': 0.0,
        #         'joint4': 0.0
        #     }
    
    def get_path_length(self, trajectory):
        """Calculate path length in joint space"""
        if not trajectory or len(trajectory.joint_trajectory.points) < 2:
            return 0.0
        
        length = 0.0
        prev_point = trajectory.joint_trajectory.points[0].positions
        
        for point in trajectory.joint_trajectory.points[1:]:
            curr_point = point.positions
            # Ensure same length
            min_len = min(len(prev_point), len(curr_point))
            length += np.linalg.norm(
                np.array(curr_point[:min_len]) - np.array(prev_point[:min_len])
            )
            prev_point = curr_point
        
        return length
    
    def get_error_message(self, error_code):
        """Convert error code to message"""
        error_map = {
            1: "SUCCESS",
            -1: "FAILURE",
            -2: "PLANNING_FAILED",
            -3: "INVALID_MOTION_PLAN",
            -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -5: "CONTROL_FAILED",
            -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -7: "TIMED_OUT",
            -8: "PREEMPTED",
            -10: "START_STATE_IN_COLLISION",
            -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -12: "GOAL_IN_COLLISION",
            -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -14: "GOAL_CONSTRAINTS_VIOLATED",
            -15: "INVALID_GROUP_NAME",
            -16: "INVALID_GOAL_CONSTRAINTS",
            -17: "INVALID_ROBOT_STATE",
            -18: "INVALID_TIMESTAMP",
            -19: "INVALID_LINK_NAME",
            -20: "INVALID_OBJECT_NAME",
            -21: "FRAME_TRANSFORM_FAILURE",
            -22: "COLLISION_CHECKING_UNAVAILABLE",
            -23: "ROBOT_STATE_STALE",
            -24: "SENSOR_INFO_STALE",
            -31: "NO_IK_SOLUTION",
            99999: "GOAL_REJECTED"
        }
        return error_map.get(error_code, f"UNKNOWN_{error_code}")
    
    def run_trial(self, goal_id, goal_config, trial_num):
        """Run a single planning trial"""
        # Get start state
        start_state = self.get_start_state()
        
        # Create planning request
        goal = MoveGroup.Goal()
        goal.request = self.create_planning_request(goal_config, start_state)
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0
        
        # Time the planning
        start_time = time.time()
        
        # Send goal
        send_future = self.move_group_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.planning_time + 5.0)
        
        if not send_future.result():
            self.get_logger().error('Failed to send goal')
            return False
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            planning_time = time.time() - start_time
            self.save_result(
                goal_id, goal_config, trial_num, False,
                planning_time, -1, "Goal rejected", "unknown", 0,
                0.0, 0, self.seed + trial_num
            )
            return False
        
        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.planning_time + 5.0)
        
        planning_time = time.time() - start_time
        
        if not result_future.result():
            self.save_result(
                goal_id, goal_config, trial_num, False,
                planning_time, -7, "TIMED_OUT", "unknown", 0,
                0.0, 0, self.seed + trial_num
            )
            return False
        
        result = result_future.result().result
        success = (result.error_code.val == 1)
    
        if success:
            self.get_logger().info(f'🔥 DEBUG: Success! Goal ID: {goal_id}, Trial: {trial_num}')
            self.get_logger().info(f'🔥 DEBUG: Has trajectory attribute? {hasattr(result, "trajectory")}')
            
            if hasattr(result, 'trajectory'):
                self.get_logger().info(f'🔥 DEBUG: Has joint_trajectory? {hasattr(result.trajectory, "joint_trajectory")}')
                
                if hasattr(result.trajectory, 'joint_trajectory'):
                    points_len = len(result.trajectory.joint_trajectory.points) if result.trajectory.joint_trajectory.points else 0
                    self.get_logger().info(f'🔥 DEBUG: Number of points in trajectory: {points_len}')
                    
                    if points_len > 0:
                        # Log first point to see structure
                        first_point = result.trajectory.joint_trajectory.points[0]
                        self.get_logger().info(f'🔥 DEBUG: First point has {len(first_point.positions)} positions')
                        self.get_logger().info(f'🔥 DEBUG: First point positions: {[f"{p:.3f}" for p in first_point.positions[:5]]}...')
            else:
                self.get_logger().info(f'🔥 DEBUG: No trajectory - error code: {result.error_code.val}')

        # Calculate path metrics if successful
        path_length = 0.0
        num_waypoints = 0
        if success and hasattr(result, 'trajectory') and result.trajectory.joint_trajectory.points:
            path_length = self.get_path_length(result.trajectory)
            num_waypoints = len(result.trajectory.joint_trajectory.points)
            self.get_logger().info(f'🔥 DEBUG: Calculated path length = {path_length:.3f}, waypoints = {num_waypoints}')
        
        self.save_result(
            goal_id, goal_config, trial_num, success,
            planning_time, result.error_code.val,
            self.get_error_message(result.error_code.val),
            result.planner_id if hasattr(result, 'planner_id') else "unknown",
            goal.request.num_planning_attempts,
            path_length, num_waypoints, self.seed + trial_num
        )
        
        # Log result
        status = "✓" if success else "✗"
        self.get_logger().info(
            f'  Trial {trial_num:3d}: {status} '
            f'({planning_time:.2f}s) - {self.get_error_message(result.error_code.val)}'
        )
        
        return success
    
    def save_result(self, goal_id, goal_config, trial_num, success,
                   planning_time, error_code, error_msg, planner_id,
                   num_attempts, path_length, num_waypoints, seed):
        """Save result to database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            INSERT INTO planning_results 
            (timestamp, config_type, goal_id, trial_number,
             success, planning_time, error_code, error_message,
             planner_id, num_planning_attempts, goal_config,
             path_length, num_waypoints, random_seed, planning_timeout)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            datetime.now().isoformat(),
            self.config_type,
            goal_id,
            trial_num,
            1 if success else 0,
            planning_time,
            error_code,
            error_msg,
            planner_id,
            num_attempts,
            json.dumps(goal_config),
            path_length,
            num_waypoints,
            seed,
            self.planning_time
        ))
        
        conn.commit()
        conn.close()
    
    def run_benchmark(self):
        """Run complete benchmark"""
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'PLANNING BENCHMARK: {self.config_type.upper()}')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Goals: {len(self.goals)}')
        self.get_logger().info(f'Trials per goal: {self.trials_per_goal}')
        self.get_logger().info(f'Total trials: {len(self.goals) * self.trials_per_goal}')
        self.get_logger().info(f'Planning timeout: {self.planning_time}s')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        
        total_success = 0
        total_trials = 0
        goal_stats = []
        
        for goal_idx, goal_config in enumerate(self.goals):
            self.get_logger().info(f'Goal {goal_idx + 1}/{len(self.goals)}')
            
            goal_successes = 0
            for trial in range(self.trials_per_goal):
                total_trials += 1
                success = self.run_trial(goal_idx, goal_config, total_trials)
                if success:
                    total_success += 1
                    goal_successes += 1
                
                # Small delay between trials
                time.sleep(0.5)
            
            goal_rate = (goal_successes / self.trials_per_goal) * 100
            goal_stats.append({
                'goal_id': goal_idx,
                'successes': goal_successes,
                'trials': self.trials_per_goal,
                'rate': goal_rate
            })
            self.get_logger().info(f'  → Goal rate: {goal_rate:.1f}% ({goal_successes}/{self.trials_per_goal})')
            self.get_logger().info('')
        
        # Summary
        overall_rate = (total_success / total_trials) * 100 if total_trials > 0 else 0
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('BENCHMARK COMPLETE')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Total trials: {total_trials}')
        self.get_logger().info(f'Total successes: {total_success}')
        self.get_logger().info(f'Overall success rate: {overall_rate:.1f}%')
        self.get_logger().info('')
        
        return {
            'total_trials': total_trials,
            'total_success': total_success,
            'overall_rate': overall_rate,
            'goal_stats': goal_stats
        }

    def verify_start_state(self):
        """Verify that the start state is collision-free"""
        start_state = self.get_start_state()
        self.get_logger().info(f'Start state for {self.config_type}:')
        for joint, value in start_state.items():
            self.get_logger().info(f'  {joint}: {value:.3f} rad ({np.degrees(value):.1f}°)')
        # In a real implementation, you'd call collision checking here
        return True


def main(args=None):
    rclpy.init(args=args)
    
    runner = PlanningBenchmarkRunner()
    runner.run_benchmark()
    
    runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()