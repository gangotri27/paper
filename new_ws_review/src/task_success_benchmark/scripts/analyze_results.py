#!/usr/bin/env python3
"""
Standalone script to analyze existing results
"""

import os
import sys
import argparse
import pandas as pd
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_success_benchmark.statistical_analyzer import StatisticalAnalyzer
from task_success_benchmark.latex_exporter import LatexExporter


def main():
    parser = argparse.ArgumentParser(description='Analyze existing task success results')
    parser.add_argument('results_file', help='Path to results CSV file')
    parser.add_argument('--output-dir', help='Output directory (default: same as input)')
    
    args = parser.parse_args()
    
    # Load results
    print(f"Loading results from: {args.results_file}")
    df = pd.read_csv(args.results_file)
    
    print(f"\nLoaded {len(df)} trials")
    print(f"Columns: {list(df.columns)}")
    
    # Set output directory
    if args.output_dir:
        output_dir = args.output_dir
    else:
        output_dir = os.path.dirname(args.results_file)
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Run analysis
    analyzer = StatisticalAnalyzer(df)
    logit_results = analyzer.run_logistic_regression()
    
    # Generate plot
    plot_path = os.path.join(output_dir, 'interaction_plot.png')
    analyzer.plot_interaction(plot_path)
    
    # Generate LaTeX tables
    exporter = LatexExporter(df, logit_results, output_dir)
    exporter.export_main_table()
    
    # Check for failures
    if len(df[~df['success'].astype(bool)]) > 0:
        exporter.export_failure_table()
    
    print(f"\n✅ Analysis complete!")
    print(f"Results saved to: {output_dir}")


if __name__ == '__main__':
    main()
