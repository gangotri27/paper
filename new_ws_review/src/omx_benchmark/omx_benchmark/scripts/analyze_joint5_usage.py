#!/usr/bin/env python3
"""
Fixed Joint5 Analysis Script
"""
import sqlite3
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import os

def analyze_joint5_usage(db_path):
    """Analyze joint5_roll values in successful trials"""
    
    if not os.path.exists(db_path):
        print(f"Database not found: {db_path}")
        return
    
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get all successful trials with their goal configurations
    cursor.execute("""
        SELECT goal_id, goal_config, planning_time 
        FROM planning_results 
        WHERE success=1 AND config_type='6dof'
        ORDER BY goal_id
    """)
    
    results = cursor.fetchall()
    
    if not results:
        print("No successful 6-DOF trials found!")
        conn.close()
        return
    
    print("=" * 60)
    print("JOINT5_ROLL UTILIZATION ANALYSIS")
    print("=" * 60)
    print(f"Total successful trials: {len(results)}")
    print()
    
    # Map goal_id to expected roll angle
    goal_roll_map = {
        1: 0, 2: 15, 3: 30, 4: 45, 5: 60, 6: 75, 7: 90
    }
    
    # Group by goal_id
    goal_data = {}
    for goal_id, goal_config_str, planning_time in results:
        try:
            goal = json.loads(goal_config_str)
        except:
            print(f"Error parsing goal config: {goal_config_str[:50]}...")
            continue
        
        # Extract joint5_roll (5th element, index 4)
        if len(goal) >= 5:
            joint5_roll = goal[4]
        else:
            print(f"Goal has only {len(goal)} joints, expected at least 5")
            continue
        
        if goal_id not in goal_data:
            goal_data[goal_id] = {
                'joint5_values': [],
                'planning_times': [],
                'count': 0
            }
        goal_data[goal_id]['joint5_values'].append(joint5_roll)
        goal_data[goal_id]['planning_times'].append(planning_time)
        goal_data[goal_id]['count'] += 1
    
    # Print analysis by goal
    joint5_by_goal = []
    expected_rolls = []
    goal_ids = []
    
    for goal_id in sorted(goal_data.keys()):
        data = goal_data[goal_id]
        avg_j5 = np.mean(data['joint5_values'])
        std_j5 = np.std(data['joint5_values'])
        expected_roll = goal_roll_map.get(goal_id, 0)
        expected_j5 = np.radians(expected_roll)
        
        print(f"Goal {goal_id} (expected roll {expected_roll}°):")
        print(f"  Successes: {data['count']}/10")
        print(f"  Avg joint5_roll: {avg_j5:.3f} rad ({np.degrees(avg_j5):.1f}°)")
        print(f"  Std dev: {std_j5:.3f} rad")
        print(f"  Expected: {expected_j5:.3f} rad ({expected_roll}°)")
        print(f"  Error: {abs(avg_j5 - expected_j5):.3f} rad")
        print(f"  Avg planning time: {np.mean(data['planning_times']):.3f}s")
        print()
        
        joint5_by_goal.append(avg_j5)
        expected_rolls.append(expected_j5)
        goal_ids.append(goal_id)
    
    if len(joint5_by_goal) > 1:
        # Calculate correlation
        correlation = np.corrcoef(joint5_by_goal, expected_rolls)[0, 1]
        print(f"Correlation between joint5_roll and target roll: R² = {correlation**2:.3f}")
        
        # Linear regression
        slope, intercept, r_value, p_value, std_err = stats.linregress(expected_rolls, joint5_by_goal)
        print(f"Linear fit: joint5 = {slope:.3f} × target + {intercept:.3f}")
        print(f"R² = {r_value**2:.3f}, p = {p_value:.6f}")
    else:
        print("Insufficient data for correlation analysis")
    
    # Create visualization
    if len(joint5_by_goal) > 1:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # Plot 1: Joint5 vs Target Roll
        ax1.scatter(expected_rolls, joint5_by_goal, s=100, alpha=0.7, color='blue')
        ax1.plot(expected_rolls, [slope*x + intercept for x in expected_rolls], 
                 'r--', label=f'Fit: y={slope:.2f}x+{intercept:.2f}')
        ax1.plot(expected_rolls, expected_rolls, 'g-', label='Ideal (y=x)')
        ax1.set_xlabel('Target Roll (rad)')
        ax1.set_ylabel('Actual Joint5_roll (rad)')
        ax1.set_title('Joint5_roll Utilization')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Success Rate by Roll Angle
        roll_angles = [goal_roll_map.get(g, 0) for g in goal_ids]
        success_rates = [goal_data[g]['count'] * 10 for g in goal_ids]  # Convert to percentage
        
        bars = ax2.bar(roll_angles, success_rates, 
                      color=['red' if r < 50 else 'green' for r in success_rates], 
                      alpha=0.7)
        ax2.set_xlabel('Target Roll Angle (degrees)')
        ax2.set_ylabel('Success Rate (%)')
        ax2.set_title('Success Rate by Roll Angle')
        ax2.set_ylim([0, 100])
        ax2.grid(True, alpha=0.3, axis='y')
        
        # Add value labels on bars
        for bar, rate in zip(bars, success_rates):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height + 2,
                    f'{rate}%', ha='center', va='bottom', fontweight='bold')
        
        plt.tight_layout()
        
        # Save figures
        output_dir = '/home/ubuntu/benchmark_test'
        os.makedirs(output_dir, exist_ok=True)
        
        fig.savefig(os.path.join(output_dir, 'joint5_utilization_fixed.png'), dpi=300)
        print(f"\nJoint5 utilization chart saved to: {os.path.join(output_dir, 'joint5_utilization_fixed.png')}")
    
    conn.close()
    
    return goal_data

def main():
    db_path = '/home/ubuntu/benchmark_test/planning_6dof_corrected.db'
    analyze_joint5_usage(db_path)

if __name__ == "__main__":
    main()