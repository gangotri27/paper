#!/usr/bin/env python3
"""
FINAL PUBLICATION FIGURE - Use your REAL data!
"""
import matplotlib.pyplot as plt
import numpy as np

# Your REAL data
widths = [0.24, 0.28, 0.32, 0.36, 0.40]
six_dof = [25.7, 65.7, 67.1, 81.4, 72.9]
five_dof = [0, 0, 0, 0, 0]

# Create the plot
plt.figure(figsize=(12, 8))

# Plot lines with markers
plt.plot(widths, six_dof, 'bo-', linewidth=3, markersize=12, label='6-DOF', markeredgecolor='black')
plt.plot(widths, five_dof, 'ro-', linewidth=3, markersize=12, label='5-DOF', markeredgecolor='black')

# Add value labels
for i, (w, s6) in enumerate(zip(widths, six_dof)):
    plt.text(w, s6+2, f'{s6:.1f}%', ha='center', va='bottom', fontsize=11, fontweight='bold', color='blue')
    plt.text(w, 5, '0%', ha='center', va='bottom', fontsize=11, fontweight='bold', color='red')

# Formatting
plt.xlabel('Corridor Width (m)', fontsize=14, fontweight='bold')
plt.ylabel('Planning Success Rate (%)', fontsize=14, fontweight='bold')
plt.title('6-DOF vs 5-DOF: Planning Success vs Corridor Width', fontsize=16, fontweight='bold')
plt.grid(True, alpha=0.3, linestyle='--')
plt.legend(fontsize=12)
plt.ylim([0, 100])
plt.xticks(widths, [f'{w:.2f}' for w in widths], fontsize=12)
plt.yticks(fontsize=12)

# Add annotation
plt.text(0.3, 50, '5-DOF: 0% at ALL widths\n(Kinematically impossible)', 
         ha='center', fontsize=12, style='italic',
         bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.3))

plt.tight_layout()
plt.savefig('/home/ubuntu/benchmark_test/final_width_sweep.png', dpi=300)
plt.savefig('/home/ubuntu/benchmark_test/final_width_sweep.pdf', dpi=300)
print("✅ FINAL FIGURE CREATED!")