#!/usr/bin/env python3
"""
Statistical Analyzer for Planning-Only Benchmark
Generates results, tables, and figures for publication
"""
import sqlite3
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import json
import os
from datetime import datetime

class PlanningAnalyzer:
    def __init__(self, db_path_5dof, db_path_6dof, output_dir='/tmp'):
        self.db_path_5dof = db_path_5dof
        self.db_path_6dof = db_path_6dof
        self.output_dir = output_dir
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        self.load_data()
        
    def load_data(self):
        """Load data from both databases"""
        # Load 5-DOF
        conn = sqlite3.connect(self.db_path_5dof)
        self.df_5dof = pd.read_sql_query(
            "SELECT * FROM planning_results", conn
        )
        conn.close()
        
        # Load 6-DOF
        conn = sqlite3.connect(self.db_path_6dof)
        self.df_6dof = pd.read_sql_query(
            "SELECT * FROM planning_results", conn
        )
        conn.close()
        
        print(f"\nLoaded {len(self.df_5dof)} 5-DOF trials")
        print(f"Loaded {len(self.df_6dof)} 6-DOF trials")
        
    def analyze_success_rates(self):
        """
        Primary analysis: Compare planning success rates
        """
        n_5dof = len(self.df_5dof)
        n_6dof = len(self.df_6dof)
        
        success_5dof = self.df_5dof['success'].sum()
        success_6dof = self.df_6dof['success'].sum()
        
        rate_5dof = success_5dof / n_5dof if n_5dof > 0 else 0
        rate_6dof = success_6dof / n_6dof if n_6dof > 0 else 0
        
        # Confidence intervals (Wilson score)
        def wilson_ci(success, n, z=1.96):
            if n == 0:
                return (0, 0)
            p = success / n
            denominator = 1 + z**2/n
            center = (p + z**2/(2*n)) / denominator
            spread = z * np.sqrt((p*(1-p)/n + z**2/(4*n**2))) / denominator
            return (max(0, center - spread), min(1, center + spread))
        
        ci_5dof = wilson_ci(success_5dof, n_5dof)
        ci_6dof = wilson_ci(success_6dof, n_6dof)
        
        # Chi-square test
        table = np.array([
            [success_5dof, n_5dof - success_5dof],
            [success_6dof, n_6dof - success_6dof]
        ])
        
        # Add small constant to avoid zero cells
        table = table + 0.5
        
        chi2, p_value, dof, expected = stats.chi2_contingency(table)
        
        # Effect sizes
        cramers_v = np.sqrt(chi2 / (np.sum(table) * (min(table.shape) - 1)))
        
        # Cohen's h
        def arcsin_transform(p):
            return 2 * np.arcsin(np.sqrt(p))
        
        h = arcsin_transform(rate_6dof) - arcsin_transform(rate_5dof)
        
        # Odds ratio
        odds_5dof = success_5dof / (n_5dof - success_5dof) if (n_5dof - success_5dof) > 0 else 0
        odds_6dof = success_6dof / (n_6dof - success_6dof) if (n_6dof - success_6dof) > 0 else 0
        odds_ratio = odds_6dof / odds_5dof if odds_5dof > 0 else float('inf')
        
        return {
            'n_5dof': int(n_5dof),
            'n_6dof': int(n_6dof),
            'success_5dof': int(success_5dof),
            'success_6dof': int(success_6dof),
            'rate_5dof': float(rate_5dof),
            'rate_6dof': float(rate_6dof),
            'ci_5dof': [float(ci_5dof[0]), float(ci_5dof[1])],
            'ci_6dof': [float(ci_6dof[0]), float(ci_6dof[1])],
            'chi_square': {
                'statistic': float(chi2),
                'p_value': float(p_value),
                'dof': int(dof)
            },
            'cramers_v': float(cramers_v),
            'cohens_h': float(abs(h)),
            'odds_ratio': float(odds_ratio) if odds_ratio != float('inf') else 'inf'
        }
    
    def analyze_planning_times(self):
        """
        Secondary analysis: Compare planning times for successful trials
        """
        # Filter successful trials
        times_5dof = self.df_5dof[self.df_5dof['success']==1]['planning_time']
        times_6dof = self.df_6dof[self.df_6dof['success']==1]['planning_time']
        
        if len(times_5dof) < 2 or len(times_6dof) < 2:
            return {
                '5dof': {'mean': None, 'std': None, 'n': len(times_5dof)},
                '6dof': {'mean': None, 'std': None, 'n': len(times_6dof)},
                't_test': None,
                'cohens_d': None,
                'mann_whitney': None
            }
        
        # T-test (Welch's)
        t_stat, t_p = stats.ttest_ind(times_6dof, times_5dof, equal_var=False)
        
        # Mann-Whitney U test (non-parametric)
        u_stat, u_p = stats.mannwhitneyu(times_6dof, times_5dof, alternative='two-sided')
        
        # Cohen's d
        mean_diff = times_6dof.mean() - times_5dof.mean()
        pooled_std = np.sqrt((times_5dof.var() + times_6dof.var()) / 2)
        cohens_d = mean_diff / pooled_std if pooled_std > 0 else 0
        
        return {
            '5dof': {
                'mean': float(times_5dof.mean()),
                'std': float(times_5dof.std()),
                'n': int(len(times_5dof)),
                'min': float(times_5dof.min()),
                'max': float(times_5dof.max()),
                'median': float(times_5dof.median())
            },
            '6dof': {
                'mean': float(times_6dof.mean()),
                'std': float(times_6dof.std()),
                'n': int(len(times_6dof)),
                'min': float(times_6dof.min()),
                'max': float(times_6dof.max()),
                'median': float(times_6dof.median())
            },
            't_test': {
                'statistic': float(t_stat),
                'p_value': float(t_p)
            },
            'mann_whitney': {
                'statistic': float(u_stat),
                'p_value': float(u_p)
            },
            'cohens_d': float(abs(cohens_d))
        }
    
    def analyze_failures(self):
        """
        Analyze failure modes
        """
        failures_5dof = self.df_5dof[self.df_5dof['success']==0]['error_message'].value_counts()
        failures_6dof = self.df_6dof[self.df_6dof['success']==0]['error_message'].value_counts()
        
        return {
            '5dof': failures_5dof.to_dict() if not failures_5dof.empty else {},
            '6dof': failures_6dof.to_dict() if not failures_6dof.empty else {}
        }
    
    def analyze_by_goal(self):
        """
        Analyze success rates per goal
        """
        # Group by goal_id
        goals_5dof = self.df_5dof.groupby('goal_id').agg({
            'success': ['count', 'sum', 'mean']
        }).reset_index()
        goals_5dof.columns = ['goal_id', 'trials', 'successes', 'rate']
        
        goals_6dof = self.df_6dof.groupby('goal_id').agg({
            'success': ['count', 'sum', 'mean']
        }).reset_index()
        goals_6dof.columns = ['goal_id', 'trials', 'successes', 'rate']
        
        return {
            '5dof': goals_5dof.to_dict('records'),
            '6dof': goals_6dof.to_dict('records')
        }
    
    def generate_latex_tables(self, success_stats, time_stats, failure_stats):
        """
        Generate LaTeX tables for publication
        """
        # Table 1: Success rates
        table1 = f"""\\begin{{table}}[htbp]
\\centering
\\caption{{Planning Success Rates for 5-DOF and 6-DOF OpenManipulator-X}}
\\label{{tab:planning_success}}
\\begin{{tabular}}{{lcccc}}
\\hline
\\textbf{{Configuration}} & \\textbf{{Trials}} & \\textbf{{Successes}} & \\textbf{{Success Rate}} & \\textbf{{95\\% CI}} \\\\
\\hline
5-DOF & {success_stats['n_5dof']} & {success_stats['success_5dof']} & {success_stats['rate_5dof']*100:.1f}\\% & [{success_stats['ci_5dof'][0]*100:.1f}\%, {success_stats['ci_5dof'][1]*100:.1f}\%] \\\\
6-DOF & {success_stats['n_6dof']} & {success_stats['success_6dof']} & {success_stats['rate_6dof']*100:.1f}\\% & [{success_stats['ci_6dof'][0]*100:.1f}\%, {success_stats['ci_6dof'][1]*100:.1f}\%] \\\\
\\hline
\\multicolumn{{5}}{{c}}{{ $\\chi^2({success_stats['chi_square']['dof']}) = {success_stats['chi_square']['statistic']:.2f}$, $p = {success_stats['chi_square']['p_value']:.4f}$ }} \\\\
\\multicolumn{{5}}{{c}}{{ Cramer's V = {success_stats['cramers_v']:.3f}, Cohen's h = {success_stats['cohens_h']:.3f} }} \\\\
\\hline
\\end{{tabular}}
\\end{{table}}"""
        
        # Table 2: Planning times
        if time_stats['t_test']:
            table2 = f"""\\begin{{table}}[htbp]
\\centering
\\caption{{Planning Times for Successful Trials}}
\\label{{tab:planning_times}}
\\begin{{tabular}}{{lcccccc}}
\\hline
\\textbf{{Config}} & \\textbf{{n}} & \\textbf{{Mean (s)}} & \\textbf{{Std (s)}} & \\textbf{{Median (s)}} & \\textbf{{Min (s)}} & \\textbf{{Max (s)}} \\\\
\\hline
5-DOF & {time_stats['5dof']['n']} & {time_stats['5dof']['mean']:.3f} & {time_stats['5dof']['std']:.3f} & {time_stats['5dof']['median']:.3f} & {time_stats['5dof']['min']:.3f} & {time_stats['5dof']['max']:.3f} \\\\
6-DOF & {time_stats['6dof']['n']} & {time_stats['6dof']['mean']:.3f} & {time_stats['6dof']['std']:.3f} & {time_stats['6dof']['median']:.3f} & {time_stats['6dof']['min']:.3f} & {time_stats['6dof']['max']:.3f} \\\\
\\hline
\\multicolumn{{7}}{{c}}{{ Welch's t-test: $t({time_stats['5dof']['n']+time_stats['6dof']['n']-2:.0f}) = {time_stats['t_test']['statistic']:.3f}$, $p = {time_stats['t_test']['p_value']:.4f}$ }} \\\\
\\multicolumn{{7}}{{c}}{{ Mann-Whitney U: $U = {time_stats['mann_whitney']['statistic']:.1f}$, $p = {time_stats['mann_whitney']['p_value']:.4f}$, Cohen's d = {time_stats['cohens_d']:.3f}$ }} \\\\
\\hline
\\end{{tabular}}
\\end{{table}}"""
        else:
            table2 = "% Insufficient data for planning time comparison"
        
        # Table 3: Failure modes
        table3 = "\\begin{table}[htbp]\n\\centering\n\\caption{Failure Mode Distribution}\n\\label{tab:failures}\n\\begin{tabular}{lcc}\n\\hline\n\\textbf{Error Type} & \\textbf{5-DOF} & \\textbf{6-DOF} \\\\\n\\hline\n"
        
        all_errors = set(failure_stats['5dof'].keys()) | set(failure_stats['6dof'].keys())
        for error in sorted(all_errors):
            count_5dof = failure_stats['5dof'].get(error, 0)
            count_6dof = failure_stats['6dof'].get(error, 0)
            table3 += f"{error} & {count_5dof} & {count_6dof} \\\\\n"
        
        table3 += "\\hline\n\\end{tabular}\n\\end{table}"
        
        return table1, table2, table3
    
    def generate_figures(self):
        """
        Generate publication-quality figures
        """
        # Figure 1: Success rate bar chart with confidence intervals
        plt.figure(figsize=(8, 6))
        
        success = self.analyze_success_rates()
        
        categories = ['5-DOF', '6-DOF']
        rates = [success['rate_5dof']*100, success['rate_6dof']*100]
        errors = [
            [(success['rate_5dof'] - success['ci_5dof'][0])*100,
             (success['ci_5dof'][1] - success['rate_5dof'])*100],
            [(success['rate_6dof'] - success['ci_6dof'][0])*100,
             (success['ci_6dof'][1] - success['rate_6dof'])*100]
        ]
        
        bars = plt.bar(categories, rates, color=['#1f77b4', '#ff7f0e'], 
                      alpha=0.8, yerr=np.array(errors).T, capsize=5,
                      edgecolor='black', linewidth=1)
        
        # Add value labels
        for bar, rate in zip(bars, rates):
            height = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2., height + 2,
                    f'{rate:.1f}%', ha='center', va='bottom', fontweight='bold')
        
        plt.ylabel('Success Rate (%)', fontsize=12)
        plt.title('Planning Success Rate Comparison', fontsize=14, fontweight='bold')
        plt.ylim([0, 100])
        plt.grid(True, alpha=0.3, axis='y')
        
        # Add statistical annotation
        sig_text = f"$\\chi^2({success['chi_square']['dof']}) = {success['chi_square']['statistic']:.2f}$"
        if success['chi_square']['p_value'] < 0.001:
            sig_text += "\n$p < 0.001$ ***"
        elif success['chi_square']['p_value'] < 0.01:
            sig_text += "\n$p < 0.01$ **"
        elif success['chi_square']['p_value'] < 0.05:
            sig_text += "\n$p < 0.05$ *"
        
        sig_text += f"\nCramer's V = {success['cramers_v']:.3f}"
        
        plt.text(0.5, 85, sig_text, ha='center', fontsize=10,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8,
                         edgecolor='gray'))
        
        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/figure1_success_rates.png', dpi=300, bbox_inches='tight')
        plt.savefig(f'{self.output_dir}/figure1_success_rates.pdf', bbox_inches='tight')
        plt.close()
        
        # Figure 2: Planning time distribution (violin plot)
        plt.figure(figsize=(10, 6))
        
        times_5dof = self.df_5dof[self.df_5dof['success']==1]['planning_time']
        times_6dof = self.df_6dof[self.df_6dof['success']==1]['planning_time']
        
        if len(times_5dof) > 0 and len(times_6dof) > 0:
            # Create DataFrame for seaborn
            df_times = pd.DataFrame({
                'Planning Time (s)': pd.concat([times_5dof, times_6dof]),
                'Configuration': ['5-DOF']*len(times_5dof) + ['6-DOF']*len(times_6dof)
            })
            
            sns.violinplot(data=df_times, x='Configuration', y='Planning Time (s)',
                          palette=['#1f77b4', '#ff7f0e'], alpha=0.7)
            
            plt.title('Planning Time Distribution (Successful Trials)', 
                     fontsize=14, fontweight='bold')
            plt.grid(True, alpha=0.3, axis='y')
            
            plt.tight_layout()
            plt.savefig(f'{self.output_dir}/figure2_planning_times.png', dpi=300, bbox_inches='tight')
            plt.savefig(f'{self.output_dir}/figure2_planning_times.pdf', bbox_inches='tight')
            plt.close()
        
        # Figure 3: Failure mode pie charts
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        failures = self.analyze_failures()
        
        def plot_pie(ax, data, title):
            if data:
                labels = [k.replace('_', ' ').title() for k in data.keys()]
                sizes = list(data.values())
                # Limit to top 5 for readability
                if len(labels) > 5:
                    sorted_items = sorted(data.items(), key=lambda x: x[1], reverse=True)
                    labels = [k.replace('_', ' ').title() for k, _ in sorted_items[:4]]
                    sizes = [v for _, v in sorted_items[:4]]
                    labels.append('Other')
                    sizes.append(sum(v for _, v in sorted_items[4:]))
                
                wedges, texts, autotexts = ax.pie(sizes, labels=labels, autopct='%1.1f%%',
                                                  startangle=90)
                for autotext in autotexts:
                    autotext.set_color('white')
                    autotext.set_fontweight('bold')
            else:
                ax.text(0.5, 0.5, 'No Failures', ha='center', va='center',
                       transform=ax.transAxes, fontsize=12)
            ax.set_title(title, fontweight='bold')
        
        plot_pie(ax1, failures['5dof'], '5-DOF Failure Modes')
        plot_pie(ax2, failures['6dof'], '6-DOF Failure Modes')
        
        plt.suptitle('Failure Mode Analysis', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(f'{self.output_dir}/figure3_failure_modes.png', dpi=300, bbox_inches='tight')
        plt.savefig(f'{self.output_dir}/figure3_failure_modes.pdf', bbox_inches='tight')
        plt.close()
        
        # Figure 4: Goal-by-goal success rates (if multiple goals)
        goals = self.analyze_by_goal()
        
        if len(goals['5dof']) > 1 or len(goals['6dof']) > 1:
            plt.figure(figsize=(12, 6))
            
            # Prepare data
            goal_data = []
            for g in goals['5dof']:
                goal_data.append({'Goal': f'G{g["goal_id"]}', 'Rate': g['rate']*100, 'Config': '5-DOF'})
            for g in goals['6dof']:
                goal_data.append({'Goal': f'G{g["goal_id"]}', 'Rate': g['rate']*100, 'Config': '6-DOF'})
            
            df_goals = pd.DataFrame(goal_data)
            
            # Plot
            ax = sns.barplot(data=df_goals, x='Goal', y='Rate', hue='Config',
                            palette=['#1f77b4', '#ff7f0e'], alpha=0.7)
            
            plt.ylabel('Success Rate (%)', fontsize=12)
            plt.title('Success Rates by Goal Configuration', fontsize=14, fontweight='bold')
            plt.ylim([0, 100])
            plt.grid(True, alpha=0.3, axis='y')
            plt.legend(title='Configuration')
            
            plt.tight_layout()
            plt.savefig(f'{self.output_dir}/figure4_goal_rates.png', dpi=300, bbox_inches='tight')
            plt.savefig(f'{self.output_dir}/figure4_goal_rates.pdf', bbox_inches='tight')
            plt.close()
    
    def generate_report(self):
        """
        Generate comprehensive analysis report
        """
        success = self.analyze_success_rates()
        times = self.analyze_planning_times()
        failures = self.analyze_failures()
        goals = self.analyze_by_goal()
        
        # Generate LaTeX tables
        table1, table2, table3 = self.generate_latex_tables(success, times, failures)
        
        # Save tables
        with open(f'{self.output_dir}/table1_success_rates.tex', 'w') as f:
            f.write(table1)
        with open(f'{self.output_dir}/table2_planning_times.tex', 'w') as f:
            f.write(table2)
        with open(f'{self.output_dir}/table3_failure_modes.tex', 'w') as f:
            f.write(table3)
        
        # Generate figures
        self.generate_figures()
        
        # Generate text report
        report = f"""
================================================================================
PLANNING BENCHMARK ANALYSIS REPORT
Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
================================================================================

1. SUCCESS RATES
--------------------------------------------------------------------------------
5-DOF: {success['success_5dof']}/{success['n_5dof']} = {success['rate_5dof']*100:.1f}%
       95% CI: [{success['ci_5dof'][0]*100:.1f}%, {success['ci_5dof'][1]*100:.1f}%]

6-DOF: {success['success_6dof']}/{success['n_6dof']} = {success['rate_6dof']*100:.1f}%
       95% CI: [{success['ci_6dof'][0]*100:.1f}%, {success['ci_6dof'][1]*100:.1f}%]

2. STATISTICAL TESTS
--------------------------------------------------------------------------------
Chi-square test: χ²({success['chi_square']['dof']}) = {success['chi_square']['statistic']:.3f}
p-value = {success['chi_square']['p_value']:.6f}

Effect sizes:
  Cramer's V = {success['cramers_v']:.3f}
  Cohen's h = {success['cohens_h']:.3f}
  Odds ratio = {success['odds_ratio']}

"""
        
        if times['t_test']:
            report += f"""
3. PLANNING TIMES (successful trials only)
--------------------------------------------------------------------------------
5-DOF: n={times['5dof']['n']}, mean={times['5dof']['mean']:.3f}s, std={times['5dof']['std']:.3f}s
       median={times['5dof']['median']:.3f}s, range=[{times['5dof']['min']:.3f}, {times['5dof']['max']:.3f}]s

6-DOF: n={times['6dof']['n']}, mean={times['6dof']['mean']:.3f}s, std={times['6dof']['std']:.3f}s
       median={times['6dof']['median']:.3f}s, range=[{times['6dof']['min']:.3f}, {times['6dof']['max']:.3f}]s

Welch's t-test: t = {times['t_test']['statistic']:.3f}, p = {times['t_test']['p_value']:.4f}
Mann-Whitney U: U = {times['mann_whitney']['statistic']:.1f}, p = {times['mann_whitney']['p_value']:.4f}
Cohen's d = {times['cohens_d']:.3f}

"""
        
        report += f"""
4. FAILURE MODES
--------------------------------------------------------------------------------
5-DOF failures ({len(self.df_5dof[self.df_5dof['success']==0])} total):
"""
        for error, count in failures['5dof'].items():
            report += f"  {error}: {count}\n"
        
        report += f"""
6-DOF failures ({len(self.df_6dof[self.df_6dof['success']==0])} total):
"""
        for error, count in failures['6dof'].items():
            report += f"  {error}: {count}\n"
        
        report += f"""
5. GOAL-BY-GOAL ANALYSIS
--------------------------------------------------------------------------------
5-DOF Goals:
"""
        for g in goals['5dof']:
            report += f"  Goal {g['goal_id']}: {g['successes']}/{g['trials']} = {g['rate']*100:.1f}%\n"
        
        report += f"""
6-DOF Goals:
"""
        for g in goals['6dof']:
            report += f"  Goal {g['goal_id']}: {g['successes']}/{g['trials']} = {g['rate']*100:.1f}%\n"
        
        report += """
================================================================================
"""
        
        # Save report
        with open(f'{self.output_dir}/analysis_report.txt', 'w') as f:
            f.write(report)
        
        # Save JSON data
        with open(f'{self.output_dir}/analysis_results.json', 'w') as f:
            json.dump({
                'success_rates': success,
                'planning_times': times,
                'failures': failures,
                'goals': goals
            }, f, indent=2)
        
        print(report)
        print(f"\nResults saved to: {self.output_dir}")
        
        return report


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Analyze planning benchmark results')
    parser.add_argument('--db-5dof', required=True, help='5-DOF database path')
    parser.add_argument('--db-6dof', required=True, help='6-DOF database path')
    parser.add_argument('--output-dir', default='/tmp', help='Output directory')
    
    args = parser.parse_args()
    
    analyzer = PlanningAnalyzer(args.db_5dof, args.db_6dof, args.output_dir)
    analyzer.generate_report()


if __name__ == '__main__':
    main()