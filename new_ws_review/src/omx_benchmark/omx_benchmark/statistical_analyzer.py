#!/usr/bin/env python3
"""
Statistical Analyzer for Benchmark Results
Computes success rates, chi-square tests, effect sizes
Generates LaTeX tables for publication
"""
import rclpy
from rclpy.node import Node
import sqlite3
import numpy as np
from scipy import stats
import json
from datetime import datetime


class StatisticalAnalyzer(Node):
    def __init__(self):
        super().__init__('statistical_analyzer')
        
        self.declare_parameter('db_path', '/tmp/omx_benchmark_results.db')
        self.db_path = self.get_parameter('db_path').value
        
        self.get_logger().info(f'Analyzing results from: {self.db_path}')
    
    def load_results(self):
        """Load results from database"""
        conn = sqlite3.connect(self.db_path)
        
        # Load 5-DOF results
        cursor = conn.execute("""
            SELECT success, planning_time, error_code, error_message
            FROM benchmark_results 
            WHERE config_type = '5dof'
        """)
        self.results_5dof = cursor.fetchall()
        
        # Load 6-DOF results
        cursor = conn.execute("""
            SELECT success, planning_time, error_code, error_message
            FROM benchmark_results 
            WHERE config_type = '6dof'
        """)
        self.results_6dof = cursor.fetchall()
        
        conn.close()
        
        self.get_logger().info(f'Loaded {len(self.results_5dof)} 5-DOF trials')
        self.get_logger().info(f'Loaded {len(self.results_6dof)} 6-DOF trials')
        
        return len(self.results_5dof) > 0 and len(self.results_6dof) > 0
    
    def compute_statistics(self):
        """Compute all statistical measures"""
        # Success counts
        n_5dof = len(self.results_5dof)
        n_6dof = len(self.results_6dof)
        
        success_5dof = sum([r[0] for r in self.results_5dof])
        success_6dof = sum([r[0] for r in self.results_6dof])
        
        rate_5dof = success_5dof / n_5dof if n_5dof > 0 else 0
        rate_6dof = success_6dof / n_6dof if n_6dof > 0 else 0
        
        # Confidence intervals (Wilson score, 95%)
        def wilson_ci(success, n, z=1.96):
            if n == 0:
                return (0, 0)
            p = success / n
            denominator = 1 + z**2/n
            center = (p + z**2/(2*n)) / denominator
            spread = z * np.sqrt((p*(1-p)/n + z**2/(4*n**2))) / denominator
            return (center - spread, center + spread)
        
        ci_5dof = wilson_ci(success_5dof, n_5dof)
        ci_6dof = wilson_ci(success_6dof, n_6dof)
        
        # Chi-square test
        table = np.array([
            [success_5dof, n_5dof - success_5dof],
            [success_6dof, n_6dof - success_6dof]
        ])
        
        chi2, p_value, dof, expected = stats.chi2_contingency(table)
        
        # Cramer's V effect size
        n_total = np.sum(table)
        cramers_v = np.sqrt(chi2 / (n_total * min(table.shape) - 1))
        
        # Cohen's h effect size
        def arcsin_transform(p):
            return 2 * np.arcsin(np.sqrt(p))
        
        h = arcsin_transform(rate_6dof) - arcsin_transform(rate_5dof)
        
        # Planning times (successful trials only)
        times_5dof = [r[1] for r in self.results_5dof if r[0] == 1]
        times_6dof = [r[1] for r in self.results_6dof if r[0] == 1]
        
        time_stats = {}
        if len(times_5dof) >= 2 and len(times_6dof) >= 2:
            t_stat, t_p = stats.ttest_ind(times_6dof, times_5dof, equal_var=False)
            
            # Cohen's d for planning time
            mean_diff = np.mean(times_6dof) - np.mean(times_5dof)
            pooled_std = np.sqrt((np.var(times_5dof, ddof=1) + np.var(times_6dof, ddof=1)) / 2)
            cohens_d = mean_diff / pooled_std if pooled_std > 0 else 0
            
            time_stats = {
                '5dof': {
                    'mean': float(np.mean(times_5dof)),
                    'std': float(np.std(times_5dof, ddof=1)),
                    'n': len(times_5dof)
                },
                '6dof': {
                    'mean': float(np.mean(times_6dof)),
                    'std': float(np.std(times_6dof, ddof=1)),
                    'n': len(times_6dof)
                },
                't_test': {
                    'statistic': float(t_stat),
                    'p_value': float(t_p)
                },
                'cohens_d': float(cohens_d)
            }
        
        # Error analysis
        errors_5dof = {}
        errors_6dof = {}
        
        for r in self.results_5dof:
            if r[0] == 0:  # failed trials
                errors_5dof[r[3]] = errors_5dof.get(r[3], 0) + 1
        
        for r in self.results_6dof:
            if r[0] == 0:  # failed trials
                errors_6dof[r[3]] = errors_6dof.get(r[3], 0) + 1
        
        return {
            'success_rates': {
                '5dof': {
                    'n': n_5dof,
                    'success': int(success_5dof),
                    'rate': float(rate_5dof),
                    'ci_lower': float(ci_5dof[0]),
                    'ci_upper': float(ci_5dof[1])
                },
                '6dof': {
                    'n': n_6dof,
                    'success': int(success_6dof),
                    'rate': float(rate_6dof),
                    'ci_lower': float(ci_6dof[0]),
                    'ci_upper': float(ci_6dof[1])
                }
            },
            'chi_square': {
                'chi2': float(chi2),
                'p_value': float(p_value),
                'dof': int(dof),
                'cramers_v': float(cramers_v),
                'cohens_h': float(h)
            },
            'planning_times': time_stats,
            'error_distribution': {
                '5dof': errors_5dof,
                '6dof': errors_6dof
            }
        }
    
    def generate_latex_tables(self, stats):
        """Generate LaTeX tables for publication"""
        sr = stats['success_rates']
        cs = stats['chi_square']
        
        # Success rate table
        success_table = f"""\\begin{{table}}[htbp]
\\centering
\\caption{{Planning Success Rates for 5-DOF and 6-DOF Configurations}}
\\label{{tab:success_rates}}
\\begin{{tabular}}{{lccc}}
\\hline
\\textbf{{Configuration}} & \\textbf{{Trials}} & \\textbf{{Successes}} & \\textbf{{Success Rate (95\\% CI)}} \\\\
\\hline
5-DOF & {sr['5dof']['n']} & {sr['5dof']['success']} & {sr['5dof']['rate']*100:.1f}\\% [{sr['5dof']['ci_lower']*100:.1f}\%, {sr['5dof']['ci_upper']*100:.1f}\%] \\\\
6-DOF & {sr['6dof']['n']} & {sr['6dof']['success']} & {sr['6dof']['rate']*100:.1f}\\% [{sr['6dof']['ci_lower']*100:.1f}\%, {sr['6dof']['ci_upper']*100:.1f}\%] \\\\
\\hline
\\multicolumn{{4}}{{c}}{{$\\chi^2({cs['dof']}) = {cs['chi2']:.2f}$, $p = {cs['p_value']:.4f}$, Cramer's V = {cs['cramers_v']:.3f}$, Cohen's h = {abs(cs['cohens_h']):.3f}$}} \\\\
\\hline
\\end{{tabular}}
\\end{{table}}"""
        
        # Planning time table
        if stats['planning_times']:
            pt = stats['planning_times']
            tt = pt['t_test']
            
            time_table = f"""\\begin{{table}}[htbp]
\\centering
\\caption{{Planning Times for Successful Trials}}
\\label{{tab:planning_times}}
\\begin{{tabular}}{{lccc}}
\\hline
\\textbf{{Configuration}} & \\textbf{{n}} & \\textbf{{Mean Time (s)}} & \\textbf{{Std Dev (s)}} \\\\
\\hline
5-DOF & {pt['5dof']['n']} & {pt['5dof']['mean']:.3f} & {pt['5dof']['std']:.3f} \\\\
6-DOF & {pt['6dof']['n']} & {pt['6dof']['mean']:.3f} & {pt['6dof']['std']:.3f} \\\\
\\hline
\\multicolumn{{4}}{{c}}{{$t({pt['5dof']['n']+pt['6dof']['n']-2:.0f}) = {tt['statistic']:.3f}$, $p = {tt['p_value']:.4f}$, Cohen's d = {abs(pt['cohens_d']):.3f}$}} \\\\
\\hline
\\end{{tabular}}
\\end{{table}}"""
        else:
            time_table = "% Insufficient successful trials for time comparison"
        
        # Error table
        error_table = "\\begin{table}[htbp]\n\\centering\n\\caption{Error Distribution}\n\\label{tab:errors}\n\\begin{tabular}{lcc}\n\\hline\n\\textbf{Error Type} & \\textbf{5-DOF} & \\textbf{6-DOF} \\\\\n\\hline\n"
        
        all_errors = set(stats['error_distribution']['5dof'].keys()) | set(stats['error_distribution']['6dof'].keys())
        for error in sorted(all_errors):
            count_5dof = stats['error_distribution']['5dof'].get(error, 0)
            count_6dof = stats['error_distribution']['6dof'].get(error, 0)
            error_table += f"{error} & {count_5dof} & {count_6dof} \\\\\n"
        
        error_table += "\\hline\n\\end{tabular}\n\\end{table}"
        
        return success_table, time_table, error_table
    
    def run_analysis(self):
        """Run complete analysis"""
        if not self.load_results():
            self.get_logger().error('No results found in database')
            return
        
        stats = self.compute_statistics()
        
        # Print results
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('STATISTICAL ANALYSIS RESULTS')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        
        sr = stats['success_rates']
        self.get_logger().info('SUCCESS RATES:')
        self.get_logger().info(f'  5-DOF: {sr["5dof"]["success"]}/{sr["5dof"]["n"]} = {sr["5dof"]["rate"]*100:.1f}%')
        self.get_logger().info(f'         95% CI: [{sr["5dof"]["ci_lower"]*100:.1f}%, {sr["5dof"]["ci_upper"]*100:.1f}%]')
        self.get_logger().info(f'  6-DOF: {sr["6dof"]["success"]}/{sr["6dof"]["n"]} = {sr["6dof"]["rate"]*100:.1f}%')
        self.get_logger().info(f'         95% CI: [{sr["6dof"]["ci_lower"]*100:.1f}%, {sr["6dof"]["ci_upper"]*100:.1f}%]')
        self.get_logger().info('')
        
        cs = stats['chi_square']
        self.get_logger().info('CHI-SQUARE TEST:')
        self.get_logger().info(f'  χ²({cs["dof"]}) = {cs["chi2"]:.3f}')
        self.get_logger().info(f'  p = {cs["p_value"]:.6f}')
        self.get_logger().info(f'  Cramer\'s V = {cs["cramers_v"]:.3f}')
        self.get_logger().info(f'  Cohen\'s h = {abs(cs["cohens_h"]):.3f}')
        
        if cs['p_value'] < 0.05:
            self.get_logger().info('  → Significant difference (p < 0.05)')
        else:
            self.get_logger().info('  → No significant difference')
        self.get_logger().info('')
        
        if stats['planning_times']:
            pt = stats['planning_times']
            tt = pt['t_test']
            self.get_logger().info('PLANNING TIMES (successful trials):')
            self.get_logger().info(f'  5-DOF: {pt["5dof"]["mean"]:.3f}s ± {pt["5dof"]["std"]:.3f}s (n={pt["5dof"]["n"]})')
            self.get_logger().info(f'  6-DOF: {pt["6dof"]["mean"]:.3f}s ± {pt["6dof"]["std"]:.3f}s (n={pt["6dof"]["n"]})')
            self.get_logger().info(f'  t-test: t = {tt["statistic"]:.3f}, p = {tt["p_value"]:.4f}')
            self.get_logger().info(f'  Cohen\'s d = {abs(pt["cohens_d"]):.3f}')
            self.get_logger().info('')
        
        self.get_logger().info('ERROR DISTRIBUTION:')
        self.get_logger().info('  5-DOF failures:')
        for error, count in stats['error_distribution']['5dof'].items():
            self.get_logger().info(f'    {error}: {count}')
        self.get_logger().info('  6-DOF failures:')
        for error, count in stats['error_distribution']['6dof'].items():
            self.get_logger().info(f'    {error}: {count}')
        self.get_logger().info('')
        
        # Generate LaTeX tables
        success_table, time_table, error_table = self.generate_latex_tables(stats)
        
        with open('/tmp/success_table.tex', 'w') as f:
            f.write(success_table)
        with open('/tmp/time_table.tex', 'w') as f:
            f.write(time_table)
        with open('/tmp/error_table.tex', 'w') as f:
            f.write(error_table)
        
        self.get_logger().info('LaTeX tables saved to /tmp/')
        self.get_logger().info('  - /tmp/success_table.tex')
        self.get_logger().info('  - /tmp/time_table.tex')
        self.get_logger().info('  - /tmp/error_table.tex')
        self.get_logger().info('')
        
        # Save stats as JSON
        with open('/tmp/statistical_analysis.json', 'w') as f:
            json.dump(stats, f, indent=2)
        
        return stats


def main(args=None):
    rclpy.init(args=args)
    
    analyzer = StatisticalAnalyzer()
    analyzer.run_analysis()
    
    analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()