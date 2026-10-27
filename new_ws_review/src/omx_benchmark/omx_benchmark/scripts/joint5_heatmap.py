#!/usr/bin/env python3
"""
Create heatmap of joint5_roll usage vs planning time
"""
import sqlite3
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
import os

def create_joint5_heatmap(db_path):
    """
    Create heatmap showing joint5_roll values used in successful trials
    """
    print("=" * 70)
    print("JOINT5_ROLL UTILIZATION HEATMAP")
    print("=" * 70)
    
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get all successful trials with their goal configurations and planning times
    cursor.execute("""
        SELECT goal_id, goal_config, planning_time
        FROM planning_results
        WHERE success=1 AND config_type='6dof'
    """)
    
    results = cursor.fetchall()
    
    if not results:
        print("No successful trials found!")
        return
    
    print(f"\n📊 Found {len(results)} successful trials")
    
    # Map goal_id to expected roll angle
    goal_roll_map = {1:0, 2:15, 3:30, 4:45, 5:60, 6:75, 7:90}
    
    # Extract data
    planning_times = []
    joint5_values = []
    roll_angles = []
    
    for goal_id, goal_config_str, planning_time in results:
        goal = json.loads(goal_config_str)
        
        # Extract joint5_roll (5th element, index 4)
        if len(goal) >= 5:
            joint5 = goal[4]
            joint5_values.append(joint5)
            planning_times.append(planning_time)
            roll_angles.append(goal_roll_map.get(goal_id, 0))
    
    joint5_values = np.array(joint5_values)
    planning_times = np.array(planning_times)
    roll_angles = np.array(roll_angles)
    
    print(f"\n📊 Joint5 range: [{joint5_values.min():.3f}, {joint5_values.max():.3f}] rad")
    print(f"📊 Planning time range: [{planning_times.min():.3f}, {planning_times.max():.3f}] s")
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))
    
    # Plot 1: Scatter plot with color by roll angle
    ax1 = plt.subplot(2, 2, 1)
    scatter = ax1.scatter(planning_times, joint5_values, c=roll_angles, 
                          cmap='viridis', s=100, alpha=0.7, edgecolor='black')
    ax1.set_xlabel('Planning Time (s)', fontsize=12)
    ax1.set_ylabel('Joint5_roll (rad)', fontsize=12)
    ax1.set_title('Joint5 Usage by Roll Angle', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax1)
    cbar.set_label('Target Roll Angle (degrees)')
    
    # Plot 2: 2D Histogram (heatmap)
    ax2 = plt.subplot(2, 2, 2)
    
    # Create 2D histogram
    heatmap, xedges, yedges = np.histogram2d(
        planning_times, joint5_values, 
        bins=[15, 15],
        range=[[0, max(planning_times)+0.5], [0, 1.7]]
    )
    
    im = ax2.imshow(heatmap.T, origin='lower', aspect='auto',
                    extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
                    cmap='hot', interpolation='nearest')
    ax2.set_xlabel('Planning Time (s)', fontsize=12)
    ax2.set_ylabel('Joint5_roll (rad)', fontsize=12)
    ax2.set_title('Density Heatmap', fontsize=14, fontweight='bold')
    cbar2 = plt.colorbar(im, ax=ax2)
    cbar2.set_label('Number of Trials')
    
    # Add expected joint5 values as reference lines
    expected_j5 = [0, 0.262, 0.524, 0.785, 1.047, 1.309, 1.571]
    for i, ej5 in enumerate(expected_j5):
        ax2.axhline(y=ej5, color='cyan', linestyle='--', alpha=0.5, linewidth=1)
        ax2.text(ax2.get_xlim()[1] - 1, ej5 + 0.03, f'{i*15}°', 
                color='cyan', fontsize=8, ha='right')
    
    # Plot 3: Kernel Density Estimation (smooth heatmap)
    ax3 = plt.subplot(2, 2, 3)
    
    # Create grid for KDE
    x_min, x_max = planning_times.min() - 0.2, planning_times.max() + 0.2
    y_min, y_max = 0, 1.7
    xx, yy = np.mgrid[x_min:x_max:100j, y_min:y_max:100j]
    positions = np.vstack([xx.ravel(), yy.ravel()])
    
    # Calculate KDE
    values = np.vstack([planning_times, joint5_values])
    kernel = gaussian_kde(values)
    f = np.reshape(kernel(positions).T, xx.shape)
    
    ax3.imshow(np.rot90(f), cmap='Blues', extent=[x_min, x_max, y_min, y_max],
               aspect='auto', alpha=0.8)
    ax3.scatter(planning_times, joint5_values, c='red', s=20, alpha=0.3)
    ax3.set_xlabel('Planning Time (s)', fontsize=12)
    ax3.set_ylabel('Joint5_roll (rad)', fontsize=12)
    ax3.set_title('Kernel Density Estimation', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.1)
    
    # Plot 4: Box plot by roll angle
    ax4 = plt.subplot(2, 2, 4)
    
    # Group joint5 values by roll angle
    data_by_roll = {}
    for i, (j5, roll) in enumerate(zip(joint5_values, roll_angles)):
        if roll not in data_by_roll:
            data_by_roll[roll] = []
        data_by_roll[roll].append(j5)
    
    # Prepare data for box plot
    box_data = []
    box_labels = []
    for roll in sorted(data_by_roll.keys()):
        box_data.append(data_by_roll[roll])
        box_labels.append(f'{roll}°')
    
    bp = ax4.boxplot(box_data, labels=box_labels, patch_artist=True)
    for patch in bp['boxes']:
        patch.set_facecolor('lightblue')
    ax4.set_xlabel('Target Roll Angle (degrees)', fontsize=12)
    ax4.set_ylabel('Joint5_roll Used (rad)', fontsize=12)
    ax4.set_title('Joint5 Distribution by Roll Angle', fontsize=14, fontweight='bold')
    ax4.grid(True, alpha=0.3, axis='y')
    
    # Add expected values
    for i, (roll, ej5) in enumerate(zip(sorted(data_by_roll.keys()), expected_j5[:len(data_by_roll)])):
        ax4.plot(i+1, ej5, 'r*', markersize=15, label='Expected' if i==0 else '')
    
    ax4.legend()
    
    plt.suptitle('Joint5_roll Utilization Analysis', fontsize=16, fontweight='bold')
    plt.tight_layout()
    
    # Save figure
    output_file = '/home/ubuntu/benchmark_test/joint5_heatmap.png'
    plt.savefig(output_file, dpi=300)
    print(f"\n✅ Heatmap saved to: {output_file}")
    
    # Create simplified version for paper
    plt.figure(figsize=(10, 8))
    
    # Main heatmap
    heatmap, xedges, yedges = np.histogram2d(
        planning_times, joint5_values, 
        bins=[20, 20],
        range=[[0, max(planning_times)+0.5], [0, 1.7]]
    )
    
    plt.imshow(heatmap.T, origin='lower', aspect='auto',
               extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
               cmap='viridis', interpolation='gaussian')
    plt.colorbar(label='Number of Successful Trials')
    plt.xlabel('Planning Time (s)', fontsize=12)
    plt.ylabel('Joint5_roll (rad)', fontsize=12)
    plt.title('Joint5_roll Utilization Heatmap\n(6-DOF Successful Trials)', 
              fontsize=14, fontweight='bold')
    
    # Add expected joint5 lines
    for i, ej5 in enumerate(expected_j5):
        plt.axhline(y=ej5, color='white', linestyle='--', alpha=0.7, linewidth=1)
        plt.text(plt.xlim()[1] - 0.8, ej5 + 0.05, f'{i*15}°', 
                color='white', fontsize=10, ha='right', fontweight='bold')
    
    # Add scatter points overlay
    plt.scatter(planning_times, joint5_values, c='red', s=30, alpha=0.3, edgecolor='white', linewidth=0.5)
    
    plt.tight_layout()
    
    # Save simplified version
    simple_output = '/home/ubuntu/benchmark_test/joint5_heatmap_simple.png'
    plt.savefig(simple_output, dpi=300)
    print(f"✅ Simplified heatmap saved to: {simple_output}")
    
    # Generate statistics
    print("\n" + "=" * 70)
    print("📊 STATISTICS BY ROLL ANGLE")
    print("=" * 70)
    
    for roll in sorted(set(roll_angles)):
        mask = roll_angles == roll
        j5_vals = joint5_values[mask]
        times = planning_times[mask]
        
        if len(j5_vals) > 0:
            print(f"\n📌 Roll {roll}° ({len(j5_vals)} trials):")
            print(f"   Joint5: mean = {j5_vals.mean():.3f} rad, std = {j5_vals.std():.3f} rad")
            print(f"   Planning time: mean = {times.mean():.3f}s, std = {times.std():.3f}s")
    
    conn.close()
    
    return

def main():
    create_joint5_heatmap('/home/ubuntu/benchmark_test/planning_6dof_corrected.db')

if __name__ == "__main__":
    main()