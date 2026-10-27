#!/usr/bin/env python3
"""
Fisher's exact test for 2x2 contingency table with zero cell
More accurate than chi-square when expected frequencies < 5
"""
import sqlite3
import numpy as np
from scipy.stats import fisher_exact, chi2_contingency
import matplotlib.pyplot as plt

def run_fisher_test(db_6dof, db_5dof):
    """
    Run Fisher's exact test on success/failure data
    """
    print("=" * 70)
    print("FISHER'S EXACT TEST ANALYSIS")
    print("=" * 70)
    
    # Load 6-DOF data
    conn_6 = sqlite3.connect(db_6dof)
    cursor = conn_6.execute("SELECT COUNT(*) FROM planning_results WHERE success=1")
    succ_6 = cursor.fetchone()[0]
    cursor = conn_6.execute("SELECT COUNT(*) FROM planning_results")
    total_6 = cursor.fetchone()[0]
    fail_6 = total_6 - succ_6
    conn_6.close()
    
    # Load 5-DOF data
    conn_5 = sqlite3.connect(db_5dof)
    cursor = conn_5.execute("SELECT COUNT(*) FROM planning_results WHERE success=1")
    succ_5 = cursor.fetchone()[0]
    cursor = conn_5.execute("SELECT COUNT(*) FROM planning_results")
    total_5 = cursor.fetchone()[0]
    fail_5 = total_5 - succ_5
    conn_5.close()
    
    print(f"\n📊 6-DOF: {succ_6}/{total_6} successes ({succ_6/total_6*100:.1f}%)")
    print(f"📊 5-DOF: {succ_5}/{total_5} successes ({succ_5/total_5*100:.1f}%)")
    print(f"\n📊 Contingency table:")
    print(f"           Success  Failure")
    print(f"6-DOF      {succ_6:6d}   {fail_6:6d}")
    print(f"5-DOF      {succ_5:6d}   {fail_5:6d}")
    
    # Create contingency table
    table = [[succ_6, fail_6], [succ_5, fail_5]]
    
    # Run chi-square for comparison
    chi2, p_chi, dof, expected = chi2_contingency(table)
    print(f"\n📊 Chi-square test: χ² = {chi2:.2f}, p = {p_chi:.6f}")
    print(f"   Expected frequencies: {expected}")
    
    # Run Fisher's exact test
    odds_ratio, p_fisher = fisher_exact(table)
    
    print(f"\n📊 Fisher's exact test:")
    print(f"   Odds ratio = {odds_ratio:.3f}")
    print(f"   p-value = {p_fisher:.6f}")
    
    # Interpretation
    print("\n" + "=" * 70)
    print("📋 INTERPRETATION")
    print("=" * 70)
    
    if p_fisher < 0.001:
        significance = "HIGHLY SIGNIFICANT"
    elif p_fisher < 0.01:
        significance = "VERY SIGNIFICANT"
    elif p_fisher < 0.05:
        significance = "SIGNIFICANT"
    else:
        significance = "NOT SIGNIFICANT"
    
    print(f"\nThe difference between 6-DOF and 5-DOF is {significance} (p = {p_fisher:.6f})")
    print(f"\nOdds ratio of {odds_ratio:.3f} means 6-DOF is {odds_ratio:.1f}x more likely to succeed")
    
    # Calculate effect size (phi coefficient)
    n_total = total_6 + total_5
    phi = (succ_6 * fail_5 - succ_5 * fail_6) / np.sqrt((succ_6+succ_5)*(fail_6+fail_5)*(succ_6+fail_6)*(succ_5+fail_5))
    print(f"\nPhi coefficient (effect size) = {phi:.3f}")
    print(f"  0.1 = small, 0.3 = medium, 0.5 = large")
    
    # Create visualization
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    # Plot 1: Success rates with confidence
    categories = ['6-DOF', '5-DOF']
    rates = [succ_6/total_6*100, succ_5/total_5*100]
    
    # Wilson confidence intervals
    def wilson_ci(success, n, z=1.96):
        if n == 0:
            return (0, 0)
        p = success / n
        denominator = 1 + z**2/n
        center = (p + z**2/(2*n)) / denominator
        spread = z * np.sqrt((p*(1-p)/n + z**2/(4*n**2))) / denominator
        return (max(0, center - spread), min(1, center + spread))
    
    ci_6 = wilson_ci(succ_6, total_6)
    ci_5 = wilson_ci(succ_5, total_5)
    
    errors = [
        [(rates[0] - ci_6[0]*100), (ci_6[1]*100 - rates[0])],
        [(rates[1] - ci_5[0]*100), (ci_5[1]*100 - rates[1])]
    ]
    
    bars = ax1.bar(categories, rates, color=['#1f77b4', '#ff7f0e'], 
                   alpha=0.8, yerr=np.array(errors).T, capsize=5)
    
    for bar, rate in zip(bars, rates):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height + 2,
                f'{rate:.1f}%', ha='center', va='bottom', fontweight='bold')
    
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_title('Planning Success Rate Comparison')
    ax1.set_ylim([0, 100])
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Add p-value annotation
    ax1.text(0.5, 90, f"Fisher's exact test\np = {p_fisher:.6f}", 
            ha='center', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Plot 2: Odds ratio visualization
    x = [0, 1]
    y = [1, odds_ratio]
    ax2.bar(x, y, color=['gray', '#2ca02c'], alpha=0.7)
    ax2.set_xticks(x)
    ax2.set_xticklabels(['5-DOF\n(baseline)', '6-DOF'])
    ax2.set_ylabel('Odds of Success\n(relative to 5-DOF)')
    ax2.set_title(f'Odds Ratio = {odds_ratio:.1f}')
    ax2.grid(True, alpha=0.3, axis='y')
    
    # Add value label
    ax2.text(1, odds_ratio + 0.5, f'{odds_ratio:.1f}x', 
            ha='center', va='bottom', fontweight='bold')
    
    plt.tight_layout()
    
    # Save figure
    output_file = '/home/ubuntu/benchmark_test/fisher_test.png'
    plt.savefig(output_file, dpi=300)
    print(f"\n✅ Analysis figure saved to: {output_file}")
    
    # Save results to file
    with open('/home/ubuntu/benchmark_test/fisher_results.txt', 'w') as f:
        f.write("=" * 70 + "\n")
        f.write("FISHER'S EXACT TEST RESULTS\n")
        f.write("=" * 70 + "\n\n")
        f.write(f"6-DOF: {succ_6}/{total_6} = {succ_6/total_6*100:.1f}%\n")
        f.write(f"5-DOF: {succ_5}/{total_5} = {succ_5/total_5*100:.1f}%\n\n")
        f.write(f"Contingency table:\n")
        f.write(f"           Success  Failure\n")
        f.write(f"6-DOF      {succ_6:6d}   {fail_6:6d}\n")
        f.write(f"5-DOF      {succ_5:6d}   {fail_5:6d}\n\n")
        f.write(f"Chi-square test: p = {p_chi:.6f}\n")
        f.write(f"Fisher's exact test: p = {p_fisher:.6f}\n")
        f.write(f"Odds ratio = {odds_ratio:.3f}\n")
        f.write(f"Phi coefficient = {phi:.3f}\n\n")
        f.write(f"CONCLUSION: The difference is {significance} (p = {p_fisher:.6f})\n")
    
    print(f"\n✅ Results saved to: /home/ubuntu/benchmark_test/fisher_results.txt")
    
    return {
        'p_fisher': p_fisher,
        'odds_ratio': odds_ratio,
        'phi': phi
    }

def main():
    run_fisher_test(
        '/home/ubuntu/benchmark_test/planning_6dof_corrected.db',
        '/home/ubuntu/benchmark_test/planning_5dof_all.db'
    )

if __name__ == "__main__":
    main()