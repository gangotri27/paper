#!/usr/bin/env python3
"""
Analyze results from corridor width sweep experiment
"""
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
import json
from datetime import datetime

def analyze_width_sweep(results_dir):
    """
    Analyze planning success across different corridor widths
    """
    print("=" * 70)
    print("CORRIDOR WIDTH SWEEP ANALYSIS")
    print("=" * 70)
    
    # Find all width subdirectories
    width_dirs = glob.glob(os.path.join(results_dir, "width_*"))
    
    if not width_dirs:
        print(f"No width directories found in {results_dir}")
        return
    
    print(f"\n📊 Found {len(width_dirs)} corridor widths:")
    
    # Store results
    widths = []
    results_6dof = []
    results_5dof = []
    
    for width_dir in sorted(width_dirs):
        # Extract corridor width from directory name
        width_str = os.path.basename(width_dir).replace("width_", "").replace("m", "")
        width = float(width_str)
        widths.append(width)
        
        print(f"\n📏 Width: {width}m")
        
        # Check 6-DOF results
        db_6 = os.path.join(width_dir, "planning_6dof.db")
        if os.path.exists(db_6):
            conn = sqlite3.connect(db_6)
            cursor = conn.execute("SELECT COUNT(*) FROM planning_results WHERE success=1")
            succ_6 = cursor.fetchone()[0]
            cursor = conn.execute("SELECT COUNT(*) FROM planning_results")
            total_6 = cursor.fetchone()[0]
            rate_6 = succ_6 / total_6 * 100 if total_6 > 0 else 0
            conn.close()
            
            print(f"  6-DOF: {succ_6}/{total_6} = {rate_6:.1f}%")
            results_6dof.append({
                'width': width,
                'successes': succ_6,
                'trials': total_6,
                'rate': rate_6
            })
        else:
            print(f"  ⚠ No 6-DOF database found")
            results_6dof.append({
                'width': width,
                'successes': 0,
                'trials': 0,
                'rate': 0
            })
        
        # Check 5-DOF results
        db_5 = os.path.join(width_dir, "planning_5dof.db")
        if os.path.exists(db_5):
            conn = sqlite3.connect(db_5)
            cursor = conn.execute("SELECT COUNT(*) FROM planning_results WHERE success=1")
            succ_5 = cursor.fetchone()[0]
            cursor = conn.execute("SELECT COUNT(*) FROM planning_results")
            total_5 = cursor.fetchone()[0]
            rate_5 = succ_5 / total_5 * 100 if total_5 > 0 else 0
            conn.close()
            
            print(f"  5-DOF: {succ_5}/{total_5} = {rate_5:.1f}%")
            results_5dof.append({
                'width': width,
                'successes': succ_5,
                'trials': total_5,
                'rate': rate_5
            })
        else:
            print(f"  ⚠ No 5-DOF database found")
            results_5dof.append({
                'width': width,
                'successes': 0,
                'trials': 0,
                'rate': 0
            })
    
    # Create visualization
    plt.figure(figsize=(12, 8))
    
    # Extract data for plotting
    x_widths = widths
    y_6dof = [r['rate'] for r in results_6dof]
    y_5dof = [r['rate'] for r in results_5dof]
    
    # Plot lines
    plt.plot(x_widths, y_6dof, 'bo-', linewidth=2, markersize=10, label='6-DOF')
    plt.plot(x_widths, y_5dof, 'ro-', linewidth=2, markersize=10, label='5-DOF')
    
    # Add markers for data points
    for i, (x, y6, y5) in enumerate(zip(x_widths, y_6dof, y_5dof)):
        plt.text(x, y6 + 2, f'{y6:.0f}%', ha='center', va='bottom', fontweight='bold', color='blue')
        if y5 > 0:
            plt.text(x, y5 + 2, f'{y5:.0f}%', ha='center', va='bottom', fontweight='bold', color='red')
    
    plt.xlabel('Corridor Width (m)', fontsize=12)
    plt.ylabel('Planning Success Rate (%)', fontsize=12)
    plt.title('Effect of Corridor Width on Planning Success', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=12)
    plt.ylim([0, 100])
    
    # Add vertical line at original width
    plt.axvline(x=0.24, color='gray', linestyle='--', alpha=0.5, label='Original width (0.24m)')
    
    plt.tight_layout()
    
    # Save figure
    output_file = os.path.join(results_dir, 'width_sweep_analysis.png')
    plt.savefig(output_file, dpi=300)
    print(f"\n✅ Analysis figure saved to: {output_file}")
    
    # Generate report
    report = []
    report.append("=" * 70)
    report.append("CORRIDOR WIDTH SWEEP ANALYSIS REPORT")
    report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("=" * 70)
    report.append("")
    
    for i, width in enumerate(widths):
        report.append(f"\n📏 Width: {width:.2f}m")
        report.append(f"  6-DOF: {results_6dof[i]['successes']}/{results_6dof[i]['trials']} = {results_6dof[i]['rate']:.1f}%")
        report.append(f"  5-DOF: {results_5dof[i]['successes']}/{results_5dof[i]['trials']} = {results_5dof[i]['rate']:.1f}%")
    
    report.append("\n" + "=" * 70)
    report.append("CONCLUSIONS")
    report.append("=" * 70)
    
    # Calculate threshold where 5-DOF first succeeds
    threshold = None
    for r in results_5dof:
        if r['rate'] > 0:
            threshold = r['width']
            break
    
    if threshold:
        report.append(f"\n• 5-DOF first achieves non-zero success at width ≥ {threshold:.2f}m")
    else:
        report.append("\n• 5-DOF achieves zero success across all tested widths")
    
    # Calculate improvement at each width
    report.append("\n• Performance gap:")
    for i, width in enumerate(widths):
        gap = results_6dof[i]['rate'] - results_5dof[i]['rate']
        report.append(f"  Width {width:.2f}m: 6-DOF leads by {gap:.1f} percentage points")
    
    report.append("\n" + "=" * 70)
    
    report_text = "\n".join(report)
    print(report_text)
    
    # Save report
    report_file = os.path.join(results_dir, 'width_sweep_report.txt')
    with open(report_file, 'w') as f:
        f.write(report_text)
    print(f"\n✅ Report saved to: {report_file}")
    
    return {
        'widths': widths,
        'results_6dof': results_6dof,
        'results_5dof': results_5dof
    }

def main():
    # Find most recent width sweep directory
    base_dir = '/home/ubuntu'
    sweep_dirs = glob.glob(os.path.join(base_dir, "benchmark_width_sweep_*"))
    
    if not sweep_dirs:
        print("No width sweep directories found!")
        return
    
    latest_dir = max(sweep_dirs, key=os.path.getctime)
    print(f"Analyzing latest sweep: {latest_dir}")
    
    analyze_width_sweep(latest_dir)

if __name__ == "__main__":
    main()