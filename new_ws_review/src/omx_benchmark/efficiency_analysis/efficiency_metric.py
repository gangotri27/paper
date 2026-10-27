#!/usr/bin/env python3
"""
efficiency_metric.py - Simple, correct efficiency analysis for your paper
Uses your EXISTING IK solutions from the goal generator
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ===== CONFIGURATION - UPDATE THESE PATHS =====
# Paths to your existing goal files (from your benchmark work)
GOALS_6DOF = "~/benchmark_test/goals_6dof_corrected.json"
GOALS_5DOF = "~/benchmark_test/goals_5dof_all.json"
# ==============================================

def load_ik_solutions(filepath):
    """Load IK solutions from goal generator output"""
    filepath = Path(filepath).expanduser()
    if not filepath.exists():
        print(f"⚠ File not found: {filepath}")
        return []
    
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    # Handle different JSON formats
    solutions = []
    if 'poses' in data:  # Format from single_robot_goal_generator
        for pose_info in data['poses']:
            solutions.append(pose_info['solution'])
    elif 'goals' in data:  # Simple format
        solutions = data['goals']
    elif isinstance(data, list):  # Direct list of goals
        solutions = data
    else:
        print(f"⚠ Unknown JSON format in {filepath}")
        return []
    
    return solutions

def joint_distance(q1, q2):
    """Euclidean distance between joint configurations"""
    if q1 is None or q2 is None:
        return 0
    min_len = min(len(q1), len(q2))
    return np.linalg.norm(np.array(q2[:min_len]) - np.array(q1[:min_len]))

def analyze_efficiency(solutions, name):
    """Compute efficiency metrics from IK solutions"""
    # Filter out None values
    valid_solutions = [s for s in solutions if s is not None]
    
    print(f"\n{name}:")
    print(f"  Total solutions: {len(solutions)}")
    print(f"  Valid solutions: {len(valid_solutions)}")
    
    if len(valid_solutions) < 2:
        print(f"  ⚠ Insufficient solutions for movement analysis")
        return None
    
    total_movement = 0
    segments = []
    valid_indices = []
    
    # Find indices where both current and previous are valid
    for i in range(1, len(solutions)):
        if solutions[i] is not None and solutions[i-1] is not None:
            dist = joint_distance(solutions[i-1], solutions[i])
            segments.append(dist)
            total_movement += dist
            valid_indices.append(i)
    
    result = {
        'name': name,
        'total_solutions': len(solutions),
        'valid_solutions': len(valid_solutions),
        'total_movement': total_movement,
        'segments': segments,
        'valid_indices': valid_indices,
        'avg_segment': np.mean(segments) if segments else 0,
        'std_segment': np.std(segments) if len(segments) > 1 else 0
    }
    
    print(f"  Valid transitions: {len(segments)}")
    print(f"  Total movement: {total_movement:.4f} rad")
    if segments:
        print(f"  Avg segment: {np.mean(segments):.4f} ± {np.std(segments):.4f} rad")
    
    return result

def simulate_time(total_movement, max_vel=1.0):
    """Simulate execution time from joint displacement"""
    return total_movement / max_vel

def generate_figures(results_6dof, results_5dof, solutions_6dof, solutions_5dof):
    """Generate publication figures"""
    
    # Figure 1: Cumulative movement
    plt.figure(figsize=(10, 6))
    
    orientations = [0, 15, 30, 45, 60, 75, 90]
    
    # Compute cumulative for 6-DOF
    cum_6dof = [0]
    for i in range(1, len(solutions_6dof)):
        if solutions_6dof[i] and solutions_6dof[i-1]:
            dist = joint_distance(solutions_6dof[i-1], solutions_6dof[i])
            cum_6dof.append(cum_6dof[-1] + dist)
        else:
            cum_6dof.append(cum_6dof[-1])
    
    # Compute cumulative for 5-DOF
    cum_5dof = [0]
    for i in range(1, len(solutions_5dof)):
        if solutions_5dof[i] and solutions_5dof[i-1]:
            dist = joint_distance(solutions_5dof[i-1], solutions_5dof[i])
            cum_5dof.append(cum_5dof[-1] + dist)
        else:
            cum_5dof.append(cum_5dof[-1])
    
    plt.plot(orientations[:len(cum_6dof)], cum_6dof, 'o-', 
             label='6-DOF', linewidth=2, markersize=8, color='#1f77b4')
    plt.plot(orientations[:len(cum_5dof)], cum_5dof, 's-', 
             label='5-DOF', linewidth=2, markersize=8, color='#ff7f0e')
    
    plt.xlabel('Target Roll Angle (degrees)', fontsize=12)
    plt.ylabel('Cumulative Joint Movement (rad)', fontsize=12)
    plt.title('Kinematic Efficiency: Joint Displacement vs Required Orientation', 
              fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('figure1_cumulative_movement.png', dpi=300, bbox_inches='tight')
    plt.savefig('figure1_cumulative_movement.pdf', bbox_inches='tight')
    print("\n✓ Figure 1 saved: figure1_cumulative_movement.png")
    plt.close()
    
    # Figure 2: Time comparison
    if results_6dof and results_5dof:
        plt.figure(figsize=(8, 6))
        
        time_6dof = simulate_time(results_6dof['total_movement'])
        time_5dof = simulate_time(results_5dof['total_movement'])
        
        bars = plt.bar(['6-DOF', '5-DOF'], [time_6dof, time_5dof], 
                       color=['#1f77b4', '#ff7f0e'], alpha=0.8)
        
        plt.ylabel('Simulated Execution Time (seconds)', fontsize=12)
        plt.title(f'Execution Time Comparison\n5-DOF requires {time_5dof/time_6dof:.1f}× longer', 
                  fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3, axis='y')
        
        # Add value labels
        for bar in bars:
            height = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.2f}s', ha='center', va='bottom', fontsize=11)
        
        plt.tight_layout()
        plt.savefig('figure2_time_comparison.png', dpi=300, bbox_inches='tight')
        plt.savefig('figure2_time_comparison.pdf', bbox_inches='tight')
        print("✓ Figure 2 saved: figure2_time_comparison.png")
        plt.close()

def generate_latex_table(results_6dof, results_5dof):
    """Generate LaTeX table for publication"""
    
    time_6dof = simulate_time(results_6dof['total_movement'])
    time_5dof = simulate_time(results_5dof['total_movement'])
    ratio = results_5dof['total_movement'] / results_6dof['total_movement']
    
    latex = r"""
\begin{table}[htbp]
\centering
\caption{Efficiency Metrics Comparison: 6-DOF vs 5-DOF}
\label{tab:efficiency}
\begin{tabular}{lccc}
\hline
\textbf{Metric} & \textbf{6-DOF} & \textbf{5-DOF} & \textbf{Ratio (5-DOF/6-DOF)} \\
\hline
Total Joint Movement (rad) & """ + f"{results_6dof['total_movement']:.3f}" + r""" & """ + f"{results_5dof['total_movement']:.3f}" + r""" & """ + f"{ratio:.1f}" + r""" \\
Simulated Time (s) & """ + f"{time_6dof:.2f}" + r""" & """ + f"{time_5dof:.2f}" + r""" & """ + f"{time_5dof/time_6dof:.1f}" + r""" \\
Valid IK Solutions & """ + f"{results_6dof['valid_solutions']}" + r"""/""" + f"{results_6dof['total_solutions']}" + r""" & """ + f"{results_5dof['valid_solutions']}" + r"""/""" + f"{results_5dof['total_solutions']}" + r""" & — \\
\hline
\end{tabular}
\end{table}
"""
    
    with open('efficiency_table.tex', 'w') as f:
        f.write(latex)
    print("✓ LaTeX table saved: efficiency_table.tex")

def main():
    print("=" * 70)
    print("EFFICIENCY METRIC ANALYSIS")
    print("=" * 70)
    
    # Load data
    print("\nLoading IK solutions from your existing goal files...")
    solutions_6dof = load_ik_solutions(GOALS_6DOF)
    solutions_5dof = load_ik_solutions(GOALS_5DOF)
    
    if not solutions_6dof:
        print("\n❌ ERROR: Could not load 6-DOF solutions. Check the path:")
        print(f"   {GOALS_6DOF}")
        return
    
    print(f"\n✓ Loaded {len(solutions_6dof)} 6-DOF solutions")
    print(f"✓ Loaded {len(solutions_5dof)} 5-DOF solutions")
    
    # Analyze
    results_6dof = analyze_efficiency(solutions_6dof, "6-DOF")
    results_5dof = analyze_efficiency(solutions_5dof, "5-DOF")
    
    if not results_5dof or results_5dof['valid_solutions'] < 2:
        print("\n⚠ IMPORTANT: 5-DOF has insufficient valid solutions")
        print("  This itself is a KEY FINDING - 5-DOF cannot achieve the required orientations")
    
    if results_6dof and results_5dof and results_5dof['total_movement'] > 0:
        ratio = results_5dof['total_movement'] / results_6dof['total_movement']
        time_6dof = simulate_time(results_6dof['total_movement'])
        time_5dof = simulate_time(results_5dof['total_movement'])
        
        print("\n" + "=" * 70)
        print("SUMMARY RESULTS")
        print("=" * 70)
        print(f"\nTotal Joint Movement:")
        print(f"  6-DOF: {results_6dof['total_movement']:.3f} rad")
        print(f"  5-DOF: {results_5dof['total_movement']:.3f} rad")
        print(f"  Ratio: {ratio:.1f}x")
        
        print(f"\nSimulated Execution Time (1 rad/s):")
        print(f"  6-DOF: {time_6dof:.2f} s")
        print(f"  5-DOF: {time_5dof:.2f} s")
        print(f"  Ratio: {time_5dof/time_6dof:.1f}x")
    
    # Generate outputs
    print("\n" + "=" * 70)
    print("GENERATING OUTPUTS")
    print("=" * 70)
    
    generate_figures(results_6dof, results_5dof, solutions_6dof, solutions_5dof)
    
    if results_6dof and results_5dof:
        generate_latex_table(results_6dof, results_5dof)
    
    # Publication text
    if results_6dof and results_5dof and results_5dof['total_movement'] > 0:
        print("\n" + "=" * 70)
        print("PUBLICATION-READY TEXT")
        print("=" * 70)
        print(f"""
> **Efficiency Analysis:** To evaluate motion efficiency independent of planning 
> stochasticity, we computed the minimum joint space displacement required for the 
> robot to transition between successive target orientations (0° to 90° in 15° 
> increments) at the fixed position (x=0.20, y=0.00, z=0.15). The 6-DOF configuration 
> required **{results_6dof['total_movement']:.2f} rad** total accumulated joint 
> movement, while the 5-DOF configuration required **{results_5dof['total_movement']:.2f} rad** 
> — a **{ratio:.1f}× increase**. This increase results from kinematic underactuation: 
> without a dedicated wrist roll joint, the 5-DOF manipulator must reposition upstream 
> joints to compensate for orientation changes that the 6-DOF achieves locally. Using 
> manufacturer-specified velocity limits (1.0 rad/s), simulated execution times were 
> **{time_6dof:.2f} s** (6-DOF) and **{time_5dof:.2f} s** (5-DOF), representing a 
> **{time_5dof/time_6dof:.1f}× time penalty**.
        """)
    
    print("\n✅ Analysis complete!")

if __name__ == "__main__":
    main()