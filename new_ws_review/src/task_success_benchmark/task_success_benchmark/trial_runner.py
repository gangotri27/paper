#!/usr/bin/env python3
"""
Runs actual trials using MoveIt planning with proper error handling
"""

import rclpy
import numpy as np
import random
import time
import yaml
import os
from task_success_benchmark.task_definition import PickPlaceTask
from task_success_benchmark.multi_stage_planner import MultiStagePlanner
from task_success_benchmark.failure_classifier import FailureClassifier


class TrialRunner:
    def __init__(self, robot_type='6dof', config_file=None):
        """
        Initialize trial runner for specified robot type
        
        Args:
            robot_type: '5dof' or '6dof'
            config_file: Path to configuration file
        """
        self.robot_type = robot_type
        
        # Load configuration
        if config_file is None:
            pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_file = os.path.join(pkg_path, 'config', 'task_params.yaml')
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.trial_config = self.config['trials']
        self.failure_classifier = FailureClassifier()
        
        # Create planner node (ROS2 node)
        rclpy.init()
        self.planner = MultiStagePlanner(robot_type, config_file)
    
    def run_trial(self, with_roll, trial_seed=None):
        """
        Run a single trial of the full task
        
        Args:
            with_roll: Whether task requires 90° roll
            trial_seed: Random seed for this trial (optional)
        
        Returns:
            Dictionary with trial results
        """
        # Generate seed if not provided
        if trial_seed is None:
            trial_seed = random.randint(1, 1000000)
        
        self.planner.get_logger().info(f"Starting trial with seed {trial_seed}")
        
        # Create task
        task = PickPlaceTask(with_roll=with_roll)
        stages = task.get_stages()
        
        # Reset to start state
        self.planner.reset_to_start()
        
        # Execute each stage
        for i, stage in enumerate(stages):
            stage_num = i + 1
            self.planner.get_logger().info(
                f"  Stage {stage_num}/{len(stages)}: {stage['name']}"
            )
            
            target_pose = (
                stage['position'][0],
                stage['position'][1],
                stage['position'][2],
                stage['orientation']
            )
            
            # Use same seed for all stages of this trial for reproducibility
            stage_seed = trial_seed + stage_num
            
            # Plan to target pose
            trajectory, error_code = self.planner.plan_to_pose(
                target_pose,
                seed=stage_seed
            )
            
            # Check for failure
            if error_code != 1:
                failure_mode = self.failure_classifier.classify(
                    error_code,
                    stage['name']
                )
                return {
                    'success': False,
                    'stage_failed': stage['name'],
                    'stage_number': stage_num,
                    'error_code': error_code,
                    'error_description': self.failure_classifier.get_error_description(error_code),
                    'failure_mode': failure_mode,
                    'stages_completed': i,
                    'seed': trial_seed,
                    'robot_type': self.robot_type,
                    'roll_required': with_roll
                }
        
        # All stages succeeded
        return {
            'success': True,
            'stages_completed': len(stages),
            'seed': trial_seed,
            'robot_type': self.robot_type,
            'roll_required': with_roll
        }
    
    def run_condition(self, with_roll, num_trials=None):
        """
        Run multiple trials for a condition
        
        Args:
            with_roll: Whether task requires 90° roll
            num_trials: Number of trials (default from config)
        
        Returns:
            List of trial result dictionaries
        """
        if num_trials is None:
            num_trials = self.trial_config['per_condition']
        
        results = []
        pause = self.trial_config['pause_between_trials']
        
        for trial in range(1, num_trials + 1):
            self.planner.get_logger().info(
                f"\nTrial {trial}/{num_trials} - {'With Roll' if with_roll else 'No Roll'}"
            )
            
            # Different seed for each trial
            trial_seed = random.randint(1, 1000000)
            
            # Run trial
            result = self.run_trial(with_roll, trial_seed)
            results.append(result)
            
            # Print status
            if result['success']:
                self.planner.get_logger().info(f"  ✓ SUCCESS [seed={trial_seed}]")
            else:
                self.planner.get_logger().info(
                    f"  ✗ FAILED: {result['failure_mode']} [seed={trial_seed}]"
                )
            
            # Pause between trials
            if trial < num_trials:
                time.sleep(pause)
        
        return results
    
    def shutdown(self):
        """Clean shutdown of ROS2 nodes"""
        self.planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
