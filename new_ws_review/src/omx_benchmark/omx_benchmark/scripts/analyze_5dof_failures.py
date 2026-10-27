#!/usr/bin/env python3
"""
Analyze 5-DOF failure modes from benchmark database
"""
import sqlite3
import json
import matplotlib.pyplot as plt
from collections import Counter
import os

def analyze_5dof_failures(db_path):
    """Analyze failure modes in 5-DOF benchmark results"""
    
    # Check if database exists
    if not os.path.exists(db_path):
        print(f"ERROR: Database not found at {db_path}")
        print("Please check the path and try again.")
        return
    
    # Connect to database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # First, let's check what columns exist
    cursor.execute("PRAGMA table_info(planning_results)")
    columns = cursor.fetchall()
    print("Database schema:")
    for col in columns:
        print(f"  {col[1]} ({col[2]})")
    print()
    
    # Determine correct column names based on schema
    # Based on your planning_benchmark_runner.py, the column is likely 'goal_config'
    goal_column = 'goal_config'  # This is what your code uses
    
    # Get all 5-DOF trials
    try:
        cursor.execute(f"""
            SELECT error_code, error_message, planning_time, {goal_column} 
            FROM planning_results 
            WHERE config_type='5dof'
        """)
    except sqlite3.OperationalError as e:
        print(f"Error with column {goal_column}: {e}")
        print("Trying alternative column names...")
        
        # Try alternative column names
        alternatives = ['goal_joints', 'goal_state', 'goal_config', 'goal']
        for alt in alternatives:
            try:
                cursor.execute(f"""
                    SELECT error_code, error_message, planning_time, {alt} 
                    FROM planning_results 
                    WHERE config_type='5dof'
                """)
                goal_column = alt
                print(f"Found working column: {alt}")
                break
            except:
                continue
    
    results = cursor.fetchall()
    
    if not results:
        print("No 5-DOF results found!")
        # List all config types in database
        cursor.execute("SELECT DISTINCT config_type FROM planning_results")
        configs = cursor.fetchall()
        print(f"Available config types: {[c[0] for c in configs]}")
        conn.close()
        return
    
    # Count failures by error type
    error_counts = Counter()
    error_details = {}
    
    for error_code, error_msg, planning_time, goal_config_str in results:
        error_counts[error_msg] += 1
        
        # Store details for each error type
        if error_msg not in error_details:
            error_details[error_msg] = {
                'count': 0,
                'planning_times': [],
                'error_codes': []
            }
        error_details[error_msg]['count'] += 1
        error_details[error_msg]['planning_times'].append(planning_time)
        error_details[error_msg]['error_codes'].append(error_code)
    
    # Print analysis
    print("=" * 60)
    print("5-DOF FAILURE MODE ANALYSIS")
    print("=" * 60)
    print(f"Total trials analyzed: {len(results)}")
    print()
    
    for error_msg, details in error_details.items():
        print(f"Error: {error_msg}")
        print(f"  Count: {details['count']} ({details['count']/len(results)*100:.1f}%)")
        print(f"  Avg planning time: {sum(details['planning_times'])/len(details['planning_times']):.3f}s")
        print()
    
    # Try to parse and show sample goal
    if results and goal_column:
        sample_goal = results[0][3]
        print(f"Sample goal ({goal_column}): {sample_goal[:100]}..." if len(str(sample_goal)) > 100 else sample_goal)
        print()
    
    # Generate failure distribution chart
    plt.figure(figsize=(10, 6))
    errors = list(error_counts.keys())
    counts = list(error_counts.values())
    
    # Shorten error messages for display
    short_errors = [e[:20] + '...' if len(e) > 20 else e for e in errors]
    
    plt.bar(range(len(errors)), counts, color='coral', alpha=0.7)
    plt.xlabel('Error Type')
    plt.ylabel('Count')
    plt.title('5-DOF Failure Mode Distribution')
    plt.xticks(range(len(errors)), short_errors, rotation=45, ha='right')
    plt.tight_layout()
    
    # Save chart
    output_dir = '/home/ubuntu/benchmark_test'
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, '5dof_failure_analysis.png')
    plt.savefig(output_file, dpi=300)
    print(f"Failure distribution chart saved to: {output_file}")
    
    conn.close()
    
    return error_details

def main():
    # Path to your 5-DOF database
    db_path = '/home/ubuntu/benchmark_test/planning_5dof_final.db'
    analyze_5dof_failures(db_path)

if __name__ == "__main__":
    main()