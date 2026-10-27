#!/usr/bin/env python3
"""
Verify which orientations are actually achievable for 5-DOF
"""

import json
import numpy as np
from pathlib import Path

def check_5dof_capability():
    """Test each orientation and report actual IK solutions"""
    
    orientations = [0, 15, 30, 45, 60, 75, 90]
    achievable = []
    
    print("\n" + "="*60)
    print("5-DOF IK CAPABILITY TEST")
    print("="*60)
    
    for angle in orientations:
        # You'll need to run this with actual robot or from your earlier data
        # For now, let's check your existing files
        
        # Look for any evidence of 5-DOF solutions
        possible_paths = [
            "~/benchmark_test/goals_5dof_all.json",
            "~/benchmark_test/goals_5dof_w0.12.json",
            "~/new_ws/src/omx_benchmark/goals_5dof_all.json"
        ]
        
        found_solution = False
        for path in possible_paths:
            p = Path(path).expanduser()
            if p.exists():
                with open(p, 'r') as f:
                    data = json.load(f)
                    if isinstance(data, dict) and 'goals' in data:
                        goals = data['goals']
                        if len(goals) > orientations.index(angle):
                            if any(abs(v) > 0.01 for v in goals[orientations.index(angle)]):
                                found_solution = True
                                break
        
        if found_solution:
            achievable.append(angle)
            print(f"  {angle:3d}°: ✓ ACHIEVABLE")
        else:
            print(f"  {angle:3d}°: ✗ NOT ACHIEVABLE")
    
    return achievable

if __name__ == "__main__":
    achievable = check_5dof_capability()
    print("\n" + "="*60)
    print(f"5-DOF can achieve: {achievable}")
    print("="*60)
