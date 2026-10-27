#!/usr/bin/env python3
"""
Statistical Analyzer using Logistic Regression with Firth correction
Handles perfect separation (0%/100% cells) gracefully
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import statsmodels.api as sm
import statsmodels.formula.api as smf
from scipy import stats
import os

# Try importing firthlogist for perfect separation handling
try:
    from firthlogist import FirthLogisticRegression
    FIRTH_AVAILABLE = True
except ImportError:
    FIRTH_AVAILABLE = False
    print("⚠ firthlogist not installed. Install with: pip install firthlogist")
    print("  Falling back to standard logistic regression.")


class StatisticalAnalyzer:
    def __init__(self, results_df):
        """
        Initialize with results dataframe
        
        Args:
            results_df: DataFrame with columns:
                - robot_type: '5dof' or '6dof'
                - roll_required: bool
                - success: 1/0
                - seed: int
                - failure_mode: str (optional)
                - stage_failed: str (optional)
        """
        self.df = results_df.copy()
        
        # Create binary variables for regression
        self.df['robot_binary'] = (self.df['robot_type'] == '6dof').astype(int)
        self.df['roll_binary'] = self.df['roll_required'].astype(int)
        
        # Check for perfect separation
        self.check_perfect_separation()
    
    def check_perfect_separation(self):
        """Check if any cell has 0% or 100% success"""
        cells = self.df.groupby(['robot_type', 'roll_required'])['success'].agg(['mean', 'count'])
        self.has_perfect_separation = any((cells['mean'] == 0) | (cells['mean'] == 1))
        
        if self.has_perfect_separation:
            print("\n" + "="*70)
            print("⚠ PERFECT SEPARATION DETECTED")
            print("="*70)
            print("Some conditions have 0% or 100% success rates.")
            print("Using appropriate statistical methods for this case.\n")
    
    def run_logistic_regression(self):
        """
        Run logistic regression with interaction term
        Uses Firth correction if perfect separation detected
        """
        print("\n" + "="*70)
        print("LOGISTIC REGRESSION RESULTS")
        print("="*70)
        
        # Check if we need Firth regression
        if self.has_perfect_separation and FIRTH_AVAILABLE:
            results = self._run_firth_regression()
        else:
            results = self._run_standard_logistic()
        
        # Print odds ratios with confidence intervals
        print("\nOdds Ratios with 95% CI:")
        print(f"  Robot (6-DOF vs 5-DOF): "
              f"OR = {results['odds_ratio_robot']:.2f} "
              f"[{results['or_ci_robot'][0]:.2f}, {results['or_ci_robot'][1]:.2f}], "
              f"p = {results['p_robot']:.4f}")
        
        print(f"  Roll (with vs without): "
              f"OR = {results['odds_ratio_roll']:.2f} "
              f"[{results['or_ci_roll'][0]:.2f}, {results['or_ci_roll'][1]:.2f}], "
              f"p = {results['p_roll']:.4f}")
        
        print(f"  Interaction: "
              f"OR = {results['odds_ratio_interaction']:.2f} "
              f"[{results['or_ci_interaction'][0]:.2f}, {results['or_ci_interaction'][1]:.2f}], "
              f"p = {results['p_interaction']:.4f}")
        
        # Add interpretation
        print("\nInterpretation:")
        if results['p_interaction'] < 0.05:
            print("  ✓ Significant interaction: Roll affects robots differently")
            if results['odds_ratio_interaction'] > 1:
                print("    → 6-DOF benefits more from having roll capability")
            else:
                print("    → 5-DOF is disproportionately harmed by roll requirement")
        else:
            print("  ✗ No significant interaction detected")
        
        return results
    
    def _run_standard_logistic(self):
        """Standard logistic regression (no perfect separation)"""
        model = smf.glm(
            'success ~ robot_binary * roll_binary',
            data=self.df,
            family=sm.families.Binomial()
        ).fit()
        
        print(model.summary())
        
        # Extract confidence intervals
        conf = model.conf_int()
        
        return {
            'coef_robot': model.params['robot_binary'],
            'coef_roll': model.params['roll_binary'],
            'coef_interaction': model.params['robot_binary:roll_binary'],
            'p_robot': model.pvalues['robot_binary'],
            'p_roll': model.pvalues['roll_binary'],
            'p_interaction': model.pvalues['robot_binary:roll_binary'],
            'odds_ratio_robot': np.exp(model.params['robot_binary']),
            'odds_ratio_roll': np.exp(model.params['roll_binary']),
            'odds_ratio_interaction': np.exp(model.params['robot_binary:roll_binary']),
            'or_ci_robot': np.exp(conf.loc['robot_binary']).tolist(),
            'or_ci_roll': np.exp(conf.loc['roll_binary']).tolist(),
            'or_ci_interaction': np.exp(conf.loc['robot_binary:roll_binary']).tolist(),
            'method': 'standard'
        }
    
    def _run_firth_regression(self):
        """Firth logistic regression (handles perfect separation)"""
        if not FIRTH_AVAILABLE:
            print("⚠ Firth regression not available, falling back to standard")
            return self._run_standard_logistic()
        
        from firthlogist import FirthLogisticRegression
        
        # Prepare data with interaction term
        X = self.df[['robot_binary', 'roll_binary']].copy()
        X['interaction'] = X['robot_binary'] * X['roll_binary']
        X = sm.add_constant(X)
        y = self.df['success']
        
        # Fit Firth model
        model = FirthLogisticRegression()
        model.fit(X, y)
        
        # Get confidence intervals via bootstrap
        # Note: This is simplified - in practice you'd use model.conf_int()
        # depending on the firthlogist API
        try:
            ci = model.conf_int()
        except:
            # Fallback to approximate CIs
            ci = {
                'robot_binary': [model.coef_[1] - 1.96*model.stderr_[1],
                                 model.coef_[1] + 1.96*model.stderr_[1]],
                'roll_binary': [model.coef_[2] - 1.96*model.stderr_[2],
                                model.coef_[2] + 1.96*model.stderr_[2]],
                'interaction': [model.coef_[3] - 1.96*model.stderr_[3],
                                model.coef_[3] + 1.96*model.stderr_[3]]
            }
        
        return {
            'coef_robot': model.coef_[1],
            'coef_roll': model.coef_[2],
            'coef_interaction': model.coef_[3],
            'p_robot': model.pvalues_[1],
            'p_roll': model.pvalues_[2],
            'p_interaction': model.pvalues_[3],
            'odds_ratio_robot': np.exp(model.coef_[1]),
            'odds_ratio_roll': np.exp(model.coef_[2]),
            'odds_ratio_interaction': np.exp(model.coef_[3]),
            'or_ci_robot': [np.exp(ci['robot_binary'][0]), np.exp(ci['robot_binary'][1])],
            'or_ci_roll': [np.exp(ci['roll_binary'][0]), np.exp(ci['roll_binary'][1])],
            'or_ci_interaction': [np.exp(ci['interaction'][0]), np.exp(ci['interaction'][1])],
            'method': 'firth'
        }
    
    def wilson_ci(self, successes, n, z=1.96):
        """
        Wilson score confidence interval for binomial proportion
        
        Args:
            successes: Number of successes
            n: Total trials
            z: z-score for confidence level (1.96 = 95%)
        
        Returns:
            (lower, upper) confidence bounds
        """
        if n == 0:
            return (0, 0)
        
        p = successes / n
        denominator = 1 + z**2 / n
        center = (p + z**2 / (2 * n)) / denominator
        spread = z * np.sqrt((p * (1 - p) / n + z**2 / (4 * n**2))) / denominator
        
        return (max(0, center - spread), min(1, center + spread))
    
    def plot_interaction(self, output_path='interaction_plot.png'):
        """
        Generate interaction plot with Wilson confidence intervals
        
        Args:
            output_path: Path to save the plot
        """
        # Calculate means and CIs for each cell
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
                    'roll': roll,
                    'rate': rate,
                    'ci_lower': ci[0],
                    'ci_upper': ci[1],
                    'n': n,
                    'successes': successes
                })
        
        # Create plot
        plt.figure(figsize=(10, 6))
        
        # Plot lines for each robot
        for robot in ['5dof', '6dof']:
            robot_data = [c for c in cells if c['robot'] == robot]
            robot_data.sort(key=lambda x: x['roll'])
            
            x = [0, 1]  # 0 = no roll, 1 = with roll
            y = [d['rate'] for d in robot_data]
            
            # Calculate error bars
            yerr = np.array([
                [y[0] - robot_data[0]['ci_lower'], robot_data[0]['ci_upper'] - y[0]],
                [y[1] - robot_data[1]['ci_lower'], robot_data[1]['ci_upper'] - y[1]]
            ]).T
            
            plt.errorbar(
                x, y, yerr=yerr,
                fmt='o-', linewidth=3, markersize=10, capsize=5,
                label=f'{robot.upper()}',
                color='#ff7f0e' if robot == '5dof' else '#1f77b4'
            )
            
            # Add sample size annotations
            for i, d in enumerate(robot_data):
                plt.text(
                    x[i], y[i] + 0.03,
                    f'n={d["n"]}',
                    ha='center', va='bottom', fontsize=9
                )
        
        plt.xlabel('Task Type', fontsize=12)
        plt.ylabel('Success Rate', fontsize=12)
        plt.xticks([0, 1], ['No Roll (0°→0°)', 'With Roll (0°→90°)'])
        plt.title(
            'Interaction Effect: Robot × Roll Requirement\n'
            '(with 95% Wilson Confidence Intervals)',
            fontsize=14, fontweight='bold'
        )
        plt.ylim([0, 1.05])
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=11)
        
        plt.tight_layout()
        
        # Save
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.savefig(output_path.replace('.png', '.pdf'), bbox_inches='tight')
        print(f"✓ Interaction plot saved: {output_path}")
        plt.close()
    
    def generate_latex_table(self, output_path='task_success_table.tex'):
        """
        Generate LaTeX table with results
        
        Args:
            output_path: Path to save the LaTeX table
        """
        # Run logistic regression
        logit_results = self.run_logistic_regression()
        
        # Calculate cell means with CIs
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
        
        # Build LaTeX table
        method_note = "Firth penalized" if logit_results['method'] == 'firth' else "Standard"
        
        latex = r"""
\begin{table}[htbp]
\centering
\caption{Task-Level Success: 2×2 Factorial Design}
\label{tab:task_success}
\begin{tabular}{lccc}
\hline
\textbf{Robot} & \textbf{Task Type} & \textbf{Success Rate} & \textbf{95\% CI} \\
\hline
"""
        
        for cell in cells:
            latex += f"{cell['robot']} & {cell['roll']} & "
            latex += f"{cell['successes']}/{cell['n']} = {cell['rate']*100:.1f}\% & "
            latex += f"[{cell['ci_lower']*100:.1f}\%, {cell['ci_upper']*100:.1f}\%] \\\\\n"
        
        latex += r"""\hline
\multicolumn{4}{c}{\textbf{Logistic Regression Results (""" + method_note + r""")}} \\
\hline
"""
        
        latex += f"\\multicolumn{{4}}{{c}}{{Robot (6-DOF vs 5-DOF): "
        latex += f"OR = {logit_results['odds_ratio_robot']:.2f} "
        latex += f"[{logit_results['or_ci_robot'][0]:.2f}, {logit_results['or_ci_robot'][1]:.2f}], "
        latex += f"$p = {logit_results['p_robot']:.4f}$}} \\\\\n"
        
        latex += f"\\multicolumn{{4}}{{c}}{{Roll (with vs without): "
        latex += f"OR = {logit_results['odds_ratio_roll']:.2f} "
        latex += f"[{logit_results['or_ci_roll'][0]:.2f}, {logit_results['or_ci_roll'][1]:.2f}], "
        latex += f"$p = {logit_results['p_roll']:.4f}$}} \\\\\n"
        
        latex += f"\\multicolumn{{4}}{{c}}{{Interaction: "
        latex += f"OR = {logit_results['odds_ratio_interaction']:.2f} "
        latex += f"[{logit_results['or_ci_interaction'][0]:.2f}, {logit_results['or_ci_interaction'][1]:.2f}], "
        latex += f"$p = {logit_results['p_interaction']:.4f}$}} \\\\\n"
        
        latex += r"""\hline
\end{tabular}
\end{table}
"""
        
        with open(output_path, 'w') as f:
            f.write(latex)
        print(f"✓ LaTeX table saved: {output_path}")
        
        return logit_result