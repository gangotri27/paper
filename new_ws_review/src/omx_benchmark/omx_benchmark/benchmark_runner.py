#!/usr/bin/env python3
"""
Benchmark Runner for 5-DOF vs 6-DOF Comparison
Records planning success/failure and timing data
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
import sqlite3
import time
from datetime import datetime
import numpy as np


class BenchmarkRunner(Node):
    def __init__(self):
        super().__init__('benchmark_runner')
        
        # Parameters
        self.declare_parameter('config_type', '6dof')  # '5dof' or '6dof'
        self.declare_parameter('num_trials', 100)
        self.declare_parameter('db_path', '/tmp/omx_benchmark_results.db')
        self.declare_parameter('goal_j1', 1.4)  # joint1 goal position
        self.declare_parameter('goal_j2', -0.25)  # joint2 goal position (adjusted)
        self.declare_parameter('goal_j3', 0.8)  # joint3 goal position (adjusted)
        self.declare_parameter('goal_j4', -0.15)  # joint4 goal position (adjusted)
        self.declare_parameter('goal_j5', 0.0)  # joint5 goal position
        
        self.config_type = self.get_parameter('config_type').value
        self.num_trials = self.get_parameter('num_trials').value
        self.db_path = self.get_parameter('db_path').value
        
        # Goal positions from parameters
        self.j1_goal = self.get_parameter('goal_j1').value
        self.j2_goal = self.get_parameter('goal_j2').value
        self.j3_goal = self.get_parameter('goal_j3').value
        self.j4_goal = self.get_parameter('goal_j4').value
        self.j5_goal = self.get_parameter('goal_j5').value
        
        # Robot-specific parameters
        if self.config_type == '6dof':
            self.group_name = 'arm'
            self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll']
            # Goal configuration from parameters
            self.goal_positions = [self.j1_goal, self.j2_goal, self.j3_goal, self.j4_goal, self.j5_goal]
        else:  # 5dof
            self.group_name = 'arm'
            self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
            # For 5-DOF, use a configuration that SHOULD fail (collision)
            self.goal_positions = [self.j1_goal, self.j2_goal, self.j3_goal, self.j4_goal]  # no joint5_roll
        
        self.get_logger().info(f'Initializing {self.config_type.upper()} benchmark')
        self.get_logger().info(f'  Group: {self.group_name}')
        self.get_logger().info(f'  Goal joints: {list(zip(self.joint_names, self.goal_positions))}')
        self.get_logger().info(f'  Trials: {self.num_trials}')
        
        # Action client
        self.move_group_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Wait for action server
        self.get_logger().info('Waiting for move_group action server...')
        if not self.move_group_action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('MoveGroup action server not available!')
            raise RuntimeError('MoveGroup action server not available')
        self.get_logger().info('✓ Connected to MoveGroup')
        
        # Initialize database
        self.init_database()
    
    def init_database(self):
        """Create SQLite database for results"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS benchmark_results (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                config_type TEXT,
                trial_number INTEGER,
                success INTEGER,
                planning_time REAL,
                error_code INTEGER,
                error_message TEXT,
                goal_joints TEXT,
                planner_id TEXT,
                num_planning_attempts INTEGER
            )
        ''')
        
        conn.commit()
        conn.close()
        self.get_logger().info(f'✓ Database ready: {self.db_path}')
    
    def create_planning_request(self):
        """Create motion planning request with JOINT CONSTRAINTS"""
        req = MotionPlanRequest()
        
        # Workspace parameters
        req.workspace_parameters.header.frame_id = "world"
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        
        req.group_name = self.group_name
        req.num_planning_attempts = 10  # INCREASED from 5
        req.allowed_planning_time = 10.0  # INCREASED from 5.0
        req.planner_id = "RRTConnectkConfigDefault"
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        
        # Goal constraints - USING JOINT CONSTRAINTS
        constraints = Constraints()
        
        # Add joint constraints for each joint
        for i, joint_name in enumerate(self.joint_names):
            joint_con = JointConstraint()
            joint_con.joint_name = joint_name
            joint_con.position = self.goal_positions[i]
            joint_con.tolerance_above = 0.01  # INCREASED tolerance
            joint_con.tolerance_below = 0.01  # INCREASED tolerance
            joint_con.weight = 1.0
            
            constraints.joint_constraints.append(joint_con)
        
        req.goal_constraints = [constraints]
        
        return req
    
    def get_error_message(self, error_code):
        """Convert error code to readable message"""
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
    
    def run_trial(self, trial_num):
        """Run a single planning trial"""
        # Create goal
        goal = MoveGroup.Goal()
        goal.request = self.create_planning_request()
        
        # Set start_state - CRITICAL: Use collision-free configuration
        if self.config_type == '6dof':
            # Start with wrist rotated 90° to avoid wall collision
            start_positions = [0.0, 0.0, 0.0, 0.0, 1.57]  # joint5_roll = 90°
        else:
            # For 5-DOF, start with arm slightly raised to avoid bottom plate
            start_positions = [0.0, 0.3, 0.3, 0.0]  # Slightly raised
        
        goal.request.start_state.joint_state.name = self.joint_names
        goal.request.start_state.joint_state.position = start_positions
        goal.request.start_state.joint_state.velocity = [0.0] * len(self.joint_names)  # Add zero velocities
        goal.request.start_state.joint_state.header.frame_id = 'world'
        goal.request.start_state.joint_state.header.stamp.sec = 0
        goal.request.start_state.joint_state.header.stamp.nanosec = 0
        
        # Planning options
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0
        
        # Send goal and measure time
        start_time = time.time()
        
        send_future = self.move_group_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            elapsed = time.time() - start_time
            self.save_result(trial_num, False, elapsed, -1, "Goal rejected", "N/A", 0)
            self.get_logger().info(f'  Trial {trial_num:3d}: ✗ REJECTED ({elapsed:.3f}s)')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        elapsed = time.time() - start_time
        
        result = result_future.result().result
        success = (result.error_code.val == 1)
        error_msg = self.get_error_message(result.error_code.val)
        
        # Get planner ID that was used
        planner_used = result_future.result().result.planner_id if hasattr(result_future.result().result, 'planner_id') else "unknown"
        
        self.save_result(trial_num, success, elapsed, result.error_code.val, error_msg, 
                        planner_used, goal.request.num_planning_attempts)
        
        if success:
            self.get_logger().info(f'  Trial {trial_num:3d}: ✓ SUCCESS ({elapsed:.3f}s)')
        else:
            self.get_logger().info(f'  Trial {trial_num:3d}: ✗ FAILED ({error_msg}) [{elapsed:.3f}s]')
        
        return success
    
    def save_result(self, trial_num, success, planning_time, error_code, error_msg, 
                   planner_id, num_attempts):
        """Save result to database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            INSERT INTO benchmark_results 
            (timestamp, config_type, trial_number, success, planning_time, 
             error_code, error_message, goal_joints, planner_id, num_planning_attempts)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            datetime.now().isoformat(),
            self.config_type,
            trial_num,
            1 if success else 0,
            planning_time,
            error_code,
            error_msg,
            str(self.goal_positions),
            planner_id,
            num_attempts
        ))
        
        conn.commit()
        conn.close()
    
    def run_benchmark(self):
        """Run all trials"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'STARTING {self.config_type.upper()} BENCHMARK')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        
        successful = 0
        start_time = time.time()
        
        for trial in range(1, self.num_trials + 1):
            if self.run_trial(trial):
                successful += 1
            # Increased delay between trials to allow system to reset
            time.sleep(1.0)  # Was 0.2
        
        total_time = time.time() - start_time
        success_rate = (successful / self.num_trials) * 100
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'BENCHMARK COMPLETE: {self.config_type.upper()}')
        self.get_logger().info(f'  Trials: {self.num_trials}')
        self.get_logger().info(f'  Successful: {successful}')
        self.get_logger().info(f'  Success Rate: {success_rate:.1f}%')
        self.get_logger().info(f'  Total Time: {total_time:.1f}s')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        
        return successful


def main(args=None):
    rclpy.init(args=args)
    
    runner = BenchmarkRunner()
    runner.run_benchmark()
    
    runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()