#!/usr/bin/env python3
"""
Main script to run the complete task success benchmark
Optimized: Single launch per robot type, both conditions in same session
"""

import os
import sys
import time
import subprocess
import argparse
import pandas as pd
import random
from pathlib import Path
import yaml

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_success_benchmark.trial_runner import TrialRunner
from task_success_benchmark.statistical_analyzer import StatisticalAnalyzer
from task_success_benchmark.latex_exporter import LatexExporter


class TaskBenchmarkRunner:
    def __init__(self):
        """Initialize benchmark runner"""
        self.new_ws = os.path.expanduser('~/new_ws')
        self.omx_ws = os.path.expanduser('~/omx_ws')
        
        # Package paths
        self.pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.config_file = os.path.join(self.pkg_path, 'config', 'task_params.yaml')
        
        # Load config
        with open(self.config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Results directory
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.results_dir = os.path.join(self.pkg_path, 'results', timestamp)
        os.makedirs(self.results_dir, exist_ok=True)
        
        # Figures directory
        self.figures_dir = os.path.join(self.pkg_path, 'figures', timestamp)
        os.makedirs(self.figures_dir, exist_ok=True)
        
        # Data storage
        self.all_results = []
        self.current_process = None
        
        print("\n" + "="*70)
        print("TASK SUCCESS BENCHMARK")
        print("="*70)
        print(f"\nConfiguration:")
        print(f"  • Trials per condition: {self.config['trials']['per_condition']}")
        print(f"  • Total trials: {self.config['trials']['per_condition'] * 4}")
        print(f"  • Design: 2×2 factorial (robot × roll)")
        print(f"  • Results directory: {self.results_dir}")
        print(f"  • Figures directory: {self.figures_dir}")
    
    def launch_robot(self, robot_type):
        """
        Launch the specified robot
        
        Args:
            robot_type: '5' for 5-DOF or '6' for 6-DOF
        """
        print(f"\n{'='*70}")
        print(f"LAUNCHING {robot_type}-DOF ROBOT")
        print(f"{'='*70}")
        
        # Select workspace based on robot type
        if robot_type == '6':
            workspace = self.new_ws
            pkg_name = 'omx_6dof_moveit'
        else:
            workspace = self.omx_ws
            pkg_name = 'omx_5dof_moveit'
        
        launch_file = os.path.join(
            workspace, 'src', pkg_name, 'launch', 'demo.launch.py'
        )
        
        if not os.path.exists(launch_file):
            print(f"ERROR: Launch file not found: {launch_file}")
            sys.exit(1)
        
        print(f"Launching: {launch_file}")
        self.current_process = subprocess.Popen(['ros2', 'launch', launch_file])
        
        print("Waiting 20 seconds for initialization...")
        time.sleep(20)
    
    def shutdown_robot(self):
        """Shutdown robot process"""
        print("\nShutting down robot...")
        if self.current_process:
            self.current_process.terminate()
            try:
                self.current_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.current_process.kill()
        
        # Kill any remaining processes
        os.system("pkill -f demo.launch.py || true")
        time.sleep(5)
        self.current_process = None
    
    def run_robot_conditions(self, robot_type):
        """
        Run both roll conditions for a single robot type
        
        Args:
            robot_type: '5' or '6' (string)
        """
        robot_full = f"{robot_type}dof"
        trials_per = self.config['trials']['per_condition']
        pause_conditions = self.config['trials']['pause_between_conditions']
        
        # Launch robot once
        self.launch_robot(robot_type)
        
        # Create trial runner
        runner = TrialRunner(robot_type=robot_full, config_file=self.config_file)
        
        # Condition 1: No Roll
        print(f"\n{'='*70}")
        print(f"CONDITION: {robot_full.upper()} - No Roll (0°→0°)")
        print(f"{'='*70}")
        
        no_roll_results = runner.run_condition(with_roll=False, num_trials=trials_per)
        self.all_results.extend(no_roll_results)
        
        # Pause between conditions
        print(f"\nPausing {pause_conditions} seconds before next condition...")
        time.sleep(pause_conditions)
        
        # Condition 2: With Roll
        print(f"\n{'='*70}")
        print(f"CONDITION: {robot_full.upper()} - With Roll (0°→90°)")
        print(f"{'='*70}")
        
        with_roll_results = runner.run_condition(with_roll=True, num_trials=trials_per)
        self.all_results.extend(with_roll_results)
        
        # Cleanup
        runner.shutdown()
        self.shutdown_robot()
        time.sleep(5)
    
    def run_full_benchmark(self):
        """Run all conditions"""
        
        # Run 5-DOF (both conditions)
        self.run_robot_conditions('5')
        
        # Run 6-DOF (both conditions)
        self.run_robot_conditions('6')
        
        # Save results
        self.save_and_analyze()
    
    def save_and_analyze(self):
        """Save results and run analysis"""
        
        # Convert to DataFrame
        df = pd.DataFrame(self.all_results)
        
        # Save raw data
        csv_path = os.path.join(self.results_dir, 'task_results.csv')
        df.to_csv(csv_path, index=False)
        print(f"\n✓ Raw results saved to: {csv_path}")
        
        # Save summary
        summary_path = os.path.join(self.results_dir, 'summary.txt')
        with open(summary_path, 'w') as f:
            f.write("TASK SUCCESS BENCHMARK SUMMARY\n")
            f.write("="*50 + "\n\n")
            
            summary = df.groupby(['robot_type', 'roll_required']).agg({
                'success': ['count', 'sum', 'mean'],
                'seed': 'nunique',
                'stages_completed': 'mean'
            }).round(3)
            
            f.write(str(summary))
        
        print(f"\nSummary saved to: {summary_path}")
        
        # Run statistical analysis
        print("\n" + "="*70)
        print("RUNNING STATISTICAL ANALYSIS")
        print("="*70)
        
        analyzer = StatisticalAnalyzer(df)
        
        # Logistic regression
        logit_results = analyzer.run_logistic_regression()
        
        # Interaction plot
        plot_path = os.path.join(self.figures_dir, 'interaction_plot.png')
        analyzer.plot_interaction(plot_path)
        
        # LaTeX tables
        exporter = LatexExporter(df, logit_results, self.results_dir)
        exporter.export_main_table()
        
        # Check if we have failures to report
        if len(df[~df['success'].astype(bool)]) > 0:
            exporter.export_failure_table()
        
        print(f"\n{'='*70}")
        print("BENCHMARK COMPLETE")
        print(f"{'='*70}")
        print(f"\nResults saved to: {self.results_dir}")
        print(f"Figures saved to: {self.figures_dir}")
        print("\nKey files:")
        print(f"  • {csv_path}")
        print(f"  • {plot_path}")
        print(f"  • {self.results_dir}/task_success_table.tex")


def main():
    parser = argparse.ArgumentParser(description='Run task success benchmark')
    parser.add_argument('--trials', type=int, default=None,
                       help='Number of trials per condition (overrides config)')
    parser.add_argument('--skip-5dof', action='store_true',
                       help='Skip 5-DOF runs (for testing/debugging)')
    parser.add_argument('--skip-6dof', action='store_true',
                       help='Skip 6-DOF runs (for testing/debugging)')
    
    args = parser.parse_args()
    
    runner = TaskBenchmarkRunner()
    
    # Override trials if specified
    if args.trials is not None:
        runner.config['trials']['per_condition'] = args.trials
    
    try:
        if args.skip_5dof and args.skip_6dof:
            print("ERROR: Cannot skip both robots!")
            sys.exit(1)
        
        if not args.skip_5dof and not args.skip_6dof:
            # Run full benchmark
            runner.run_full_benchmark()
        else:
            # Run selected robots only
            if not args.skip_5dof:
                runner.run_robot_conditions('5')
            if not args.skip_6dof:
                runner.run_robot_conditions('6')
            
            # Save and analyze
            runner.save_and_analyze()
            
    except KeyboardInterrupt:
        print("\n\nBenchmark interrupted by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        runner.shutdown_robot()
    
    print("\n✅ Done!")


if __name__ == '__main__':
    main()
