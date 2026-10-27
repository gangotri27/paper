#!/usr/bin/env python3
"""
LaTeX exporter for task success results
"""

import pandas as pd
import numpy as np
from datetime import datetime


class LatexExporter:
    def __init__(self, results_df, logit_results, output_dir='.'):
        """
        Initialize exporter
        
        Args:
            results_df: DataFrame with trial results
            logit_results: Results from logistic regression
            output_dir: Directory to save LaTeX files
        """
        self.df = results_df
        self.logit = logit_results
        self.output_dir = output_dir
    
    def wilson_ci(self, successes, n, z=1.96):
        """Wilson score confidence interval"""
        if n == 0:
            return (0, 0)
        p = successes / n
        denominator = 1 + z**2 / n
        center = (p + z**2 / (2 * n)) / denominator
        spread = z * np.sqrt((p * (1 - p) / n + z**2 / (4 * n**2))) / denominator
        return (max(0, center - spread), min(1, center + spread))
    
    def export_main_table(self, filename='task_success_table.tex'):
        """Export main results table"""
        
        # Calculate cell statistics
        cells = []
        for robot in ['5dof', '6dof']:
            for roll in [False, True]:
                data = self.df[
                    (self.df['robot_type'] == robot) & 
                    (self.df['roll_required'] == roll)
                ]
                n = len(data)
                successes = data['success'].sum()
                rate = successes / n if n > 0 else 0
                ci = self.wilson_ci(successes, n)
                
                cells.append({
                    'robot': robot,
                    'roll': 'No Roll' if not roll else 'With Roll',
                    'n': n,
                    'successes': successes,
                    'rate': rate,
                    'ci_lower': ci[0],
                    'ci_upper': ci[1]
                })
        
        # Build LaTeX
        latex = r"""% Auto-generated LaTeX table
% Generated: """ + datetime.now().strftime('%Y-%m-%d %H:%M:%S') + r"""

\\begin{table}[htbp]
\\centering
\\caption{Task-Level Success: 2×2 Factorial Design}
\\label{tab:task_success}
\\begin{tabular}{lccc}
\\hline
\\textbf{Robot} & \\textbf{Task Type} & \\textbf{Success Rate} & \\textbf{95\\% CI} \\\\
\\hline
"""
        
        for cell in cells:
            latex += f"{cell['robot']} & {cell['roll']} & "
            latex += f"{cell['successes']}/{cell['n']} = {cell['rate']*100:.1f}\\% & "
            latex += f"[{cell['ci_lower']*100:.1f}\\%, {cell['ci_upper']*100:.1f}\\%] \\\\\n"
        
        latex += r"""\hline
\multicolumn{4}{c}{\textbf{Logistic Regression Results}} \\
\hline
"""
        
        latex += f"\\multicolumn{{4}}{{c}}{{Robot (6-DOF vs 5-DOF): "
        latex += f"OR = {self.logit['odds_ratio_robot']:.2f} "
        latex += f"[{self.logit['or_ci_robot'][0]:.2f}, {self.logit['or_ci_robot'][1]:.2f}], "
        latex += f"$p = {self.logit['p_robot']:.4f}$}} \\\\\n"
        
        latex += f"\\multicolumn{{4}}{{c}}{{Roll (with vs without): "
        latex += f"OR = {self.logit['odds_ratio_roll']:.2f} "
        latex += f"[{self.logit['or_ci_roll'][0]:.2f}, {self.logit['or_ci_roll'][1]:.2f}], "
        latex += f"$p = {self.logit['p_roll']:.4f}$}} \\\\\n"
        
        latex += f"\\multicolumn{{4}}{{c}}{{Interaction: "
        latex += f"OR = {self.logit['odds_ratio_interaction']:.2f} "
        latex += f"[{self.logit['or_ci_interaction'][0]:.2f}, {self.logit['or_ci_interaction'][1]:.2f}], "
        latex += f"$p = {self.logit['p_interaction']:.4f}$}} \\\\\n"
        
        latex += r"""\hline
\end{tabular}
\end{table}
"""
        
        output_path = f"{self.output_dir}/{filename}"
        with open(output_path, 'w') as f:
            f.write(latex)
        print(f"✓ LaTeX table saved: {output_path}")
    
    def export_failure_table(self, filename='failure_distribution.tex'):
        """Export failure distribution table"""
        
        # Get failure modes for non-successful trials
        failures = self.df[~self.df['success'].astype(bool)]
        
        if len(failures) == 0:
            print("No failures to report")
            return
        
        # Count by robot and failure mode
        failure_counts = failures.groupby(['robot_type', 'failure_mode']).size().reset_index(name='count')
        
        latex = r"""% Failure Distribution Table
% Generated: """ + datetime.now().strftime('%Y-%m-%d %H:%M:%S') + r"""

\\begin{table}[htbp]
\\centering
\\caption{Failure Distribution by Robot and Failure Mode}
\\label{tab:failure_distribution}
\\begin{tabular}{lcc}
\\hline
\\textbf{Robot} & \\textbf{Failure Mode} & \\textbf{Count} \\\\
\\hline
"""
        
        for _, row in failure_counts.iterrows():
            latex += f"{row['robot_type']} & {row['failure_mode']} & {row['count']} \\\\\n"
        
        latex += r"""\hline
\end{tabular}
\end{table}
"""
        
        output_path = f"{self.output_dir}/{filename}"
        with open(output_path, 'w') as f:
            f.write(latex)
        print(f"✓ Failure distribution table saved: {output_path}")
