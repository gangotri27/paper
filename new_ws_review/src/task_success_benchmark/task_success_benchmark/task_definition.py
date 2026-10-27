#!/usr/bin/env python3
"""
Defines the multi-stage pick and place task
"""

import yaml
import os

class PickPlaceTask:
    def __init__(self, with_roll=False, config_file=None):
        """
        Initialize task with given configuration
        
        Args:
            with_roll: Whether task requires 90° roll
            config_file: Path to YAML configuration file
        """
        self.with_roll = with_roll
        
        # Load configuration
        if config_file is None:
            # Default to package config
            pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_file = os.path.join(pkg_path, 'config', 'task_params.yaml')
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Extract task parameters
        task_cfg = self.config['task']
        self.pick_position = (
            task_cfg['pick']['x'],
            task_cfg['pick']['y'],
            task_cfg['pick']['z']
        )
        self.place_position = (
            task_cfg['place']['x'],
            task_cfg['place']['y'],
            task_cfg['place']['z']
        )
        self.lift_height = task_cfg['lift_height']
        
        # Set orientations based on roll requirement
        if with_roll:
            self.pick_orientation = task_cfg['orientations']['with_roll']['pick']
            self.place_orientation = task_cfg['orientations']['with_roll']['place']
        else:
            self.pick_orientation = task_cfg['orientations']['no_roll']['pick']
            self.place_orientation = task_cfg['orientations']['no_roll']['place']
    
    def get_stages(self):
        """
        Return sequence of poses for the task
        
        Returns:
            List of dictionaries, each with:
            - name: stage name
            - position: (x, y, z) tuple
            - orientation: roll angle in radians
            - type: 'waypoint' or 'goal'
        """
        stages = []
        
        # Stage 1: Approach pick (above)
        stages.append({
            'name': 'approach_pick',
            'position': (self.pick_position[0], self.pick_position[1], self.lift_height),
            'orientation': self.pick_orientation,
            'type': 'waypoint'
        })
        
        # Stage 2: Pick
        stages.append({
            'name': 'pick',
            'position': self.pick_position,
            'orientation': self.pick_orientation,
            'type': 'goal'
        })
        
        # Stage 3: Lift
        stages.append({
            'name': 'lift',
            'position': (self.pick_position[0], self.pick_position[1], self.lift_height),
            'orientation': self.pick_orientation,
            'type': 'waypoint'
        })
        
        # Stage 4: Transport midpoint (with gradual rotation if needed)
        if self.with_roll:
            # Midpoint orientation: 45° (half of 90°)
            mid_orientation = 0.785398  # 45 degrees
        else:
            mid_orientation = self.pick_orientation
        
        stages.append({
            'name': 'transport_mid',
            'position': (0.20, 0.00, self.lift_height),  # midpoint
            'orientation': mid_orientation,
            'type': 'waypoint'
        })
        
        # Stage 5: Approach place
        stages.append({
            'name': 'approach_place',
            'position': (self.place_position[0], self.place_position[1], self.lift_height),
            'orientation': self.place_orientation,
            'type': 'waypoint'
        })
        
        # Stage 6: Place
        stages.append({
            'name': 'place',
            'position': self.place_position,
            'orientation': self.place_orientation,
            'type': 'goal'
        })
        
        # Stage 7: Retreat
        stages.append({
            'name': 'retreat',
            'position': (self.place_position[0], self.place_position[1], self.lift_height),
            'orientation': self.place_orientation,
            'type': 'waypoint'
        })
        
        return stages
    
    def get_stage_count(self):
        """Return number of stages"""
        return len(self.get_stages())
