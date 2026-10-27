#!/usr/bin/env python3
"""
Multi-stage motion planner using MoveIt services
CORRECTED: Position constraint with proper primitive definition
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
import numpy as np
import yaml
import os
import time


class MultiStagePlanner(Node):
    def __init__(self, robot_type='6dof', config_file=None):
        """
        Initialize planner for specified robot type
        
        Args:
            robot_type: '5dof' or '6dof'
            config_file: Path to configuration file
        """
        super().__init__(f'multi_stage_planner_{robot_type}')
        self.robot_type = robot_type
        
        # Load configuration
        if config_file is None:
            pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_file = os.path.join(pkg_path, 'config', 'task_params.yaml')
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Extract parameters
        self.planning_config = self.config['planning']
        self.robot_config = self.config['robot']
        
        # Robot-specific settings
        self.joint_names = self.robot_config['joints'][robot_type]
        self.ee_link = self.robot_config['ee_links'][robot_type]
        
        # Start state from config
        start_cfg = self.robot_config['start_state']
        if robot_type == '6dof':
            self.start_state = [
                start_cfg['joint1'],
                start_cfg['joint2'],
                start_cfg['joint3'],
                start_cfg['joint4'],
                start_cfg['joint5_roll']
            ]
        else:
            self.start_state = [
                start_cfg['joint1'],
                start_cfg['joint2'],
                start_cfg['joint3'],
                start_cfg['joint4']
            ]
        
        self.current_joint_state = self.start_state.copy()
        
        # MoveIt service client
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # Wait for service
        self.get_logger().info(f'Waiting for plan_kinematic_path service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available!')
            raise RuntimeError('plan_kinematic_path service not available')
        self.get_logger().info('✓ Connected to plan_kinematic_path service')
        
        # Verify orientation convention
        self.get_logger().info(f'Using end-effector link: {self.ee_link}')
        self.get_logger().info('Orientation: Roll about X-axis, Pitch about Y-axis, Yaw about Z-axis')
        self.get_logger().info('Note: OMPL seeds are logged for reproducibility, but stochastic variability is retained')
    
    def set_ompl_seed(self, seed):
        """
        Log OMPL random seed (for reproducibility tracking)
        Note: Actual OMPL determinism requires setting seed in ompl_planning.yaml
        """
        self.get_logger().debug(f"Trial seed: {seed}")
        # No runtime parameter setting - documented limitation
    
    def quaternion_from_roll(self, roll_rad):
        """
        Convert roll angle to quaternion (roll about X-axis)
        Verified against OpenMANIPULATOR-X URDF convention
        """
        return Quaternion(
            x=np.sin(roll_rad/2),
            y=0.0,
            z=0.0,
            w=np.cos(roll_rad/2)
        )
    
    def plan_to_pose(self, target_pose, seed=None):
        """
        Plan from current state to target pose
        
        Args:
            target_pose: (x, y, z, roll) tuple
            seed: Optional random seed for logging
        
        Returns:
            trajectory: Joint trajectory if successful, None otherwise
            error_code: MoveIt error code
        """
        # Log seed if provided
        if seed is not None:
            self.set_ompl_seed(seed)
        
        # Create motion plan request
        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.allowed_planning_time = self.planning_config['timeout_per_stage']
        request.planner_id = self.planning_config['planner_id']
        request.num_planning_attempts = self.planning_config['num_planning_attempts']
        
        # IMPORTANT: Set the workspace parameters (often required)
        request.workspace_parameters.header.frame_id = 'world'
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -0.5
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0
        
        # Set start state
        request.start_state.joint_state.name = self.joint_names
        request.start_state.joint_state.position = self.current_joint_state
        request.start_state.joint_state.header.frame_id = 'world'
        
        # Create goal constraints
        constraints = Constraints()
        tol = self.planning_config['tolerances']
        
        # ===== POSITION CONSTRAINT =====
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'world'
        pos_constraint.link_name = self.ee_link
        
        # Define constraint region as a sphere with radius = position tolerance
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [tol['position']]  # sphere radius
        pos_constraint.constraint_region.primitives.append(primitive)
        
        # Set target position
        target_pose_msg = Pose()
        target_pose_msg.position.x = target_pose[0]
        target_pose_msg.position.y = target_pose[1]
        target_pose_msg.position.z = target_pose[2]
        pos_constraint.constraint_region.primitive_poses.append(target_pose_msg)
        
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        
        # ===== ORIENTATION CONSTRAINT =====
        # Create orientation constraint with proper tolerance fields
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'world'
        ori_constraint.link_name = self.ee_link
        ori_constraint.orientation = self.quaternion_from_roll(target_pose[3])
        
        # Set the tolerances
        ori_constraint.absolute_x_axis_tolerance = tol['roll']
        ori_constraint.absolute_y_axis_tolerance = tol['pitch']
        ori_constraint.absolute_z_axis_tolerance = tol['yaw']
        
        # Set parameterization
        ori_constraint.parameterization = OrientationConstraint.XYZ_EULER_ANGLES
        
        ori_constraint.weight = 1.0
        constraints.orientation_constraints.append(ori_constraint)
        
        request.goal_constraints = [constraints]
        
        # Log the request for debugging
        self.get_logger().debug(f"Sending planning request for {target_pose}")
        
        # Create the service request wrapper
        from moveit_msgs.srv import GetMotionPlan
        
        service_request = GetMotionPlan.Request()
        service_request.motion_plan_request = request

        # Add this debug block
        self.get_logger().info(f"Request group: {request.group_name}")
        self.get_logger().info(f"Number of constraints: {len(request.goal_constraints)}")
        self.get_logger().info(f"Start state joints: {request.start_state.joint_state.name}")
        self.get_logger().info(f"Start state positions: {request.start_state.joint_state.position}")

        # Check if constraints are properly formed
        if len(request.goal_constraints) > 0:
            constraint = request.goal_constraints[0]
            self.get_logger().info(f"Position constraints: {len(constraint.position_constraints)}")
            self.get_logger().info(f"Orientation constraints: {len(constraint.orientation_constraints)}")
            
            # Check orientation constraint details
            if len(constraint.orientation_constraints) > 0:
                ori = constraint.orientation_constraints[0]
                self.get_logger().info(f"Orientation link: {ori.link_name}")
                self.get_logger().info(f"Orientation: {ori.orientation}")
                self.get_logger().info(f"X tolerance: {ori.absolute_x_axis_tolerance}")
                self.get_logger().info(f"Y tolerance: {ori.absolute_y_axis_tolerance}")
                self.get_logger().info(f"Z tolerance: {ori.absolute_z_axis_tolerance}")
                self.get_logger().info(f"Weight: {ori.weight}")

        # Call service
        future = self.plan_client.call_async(service_request)
        rclpy.spin_until_future_complete(
            self, future,
            timeout_sec=self.planning_config['timeout_per_stage'] + 2.0
        )
        
        if future.result() is None:
            self.get_logger().error("Planning service returned None")
            return None, -7  # TIMEOUT
        
        response = future.result()
        error_code = response.motion_plan_response.error_code.val
        
        if error_code == 1:  # SUCCESS
            trajectory = response.motion_plan_response.trajectory
            if trajectory.joint_trajectory.points:
                # Update current state to last point
                self.current_joint_state = trajectory.joint_trajectory.points[-1].positions
            return trajectory, 1
        else:
            self.get_logger().warn(f"Planning failed with error code: {error_code}")
            return None, error_code
    
    def reset_to_start(self):
        """Reset planner to start state"""
        self.current_joint_state = self.start_state.copy()