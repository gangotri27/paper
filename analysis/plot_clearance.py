#!/usr/bin/env python3
"""
Plot minimum passable gap vs wrist roll from the CSVs written by clearance_curve.py.
Sized for one IEEE column (3.5 in wide). Run in the venv (needs matplotlib).

  python3 plot_clearance.py --six ~/analysis/clearance_6dof.csv --five ~/analysis/clearance_5dof.csv --out ~/analysis/fig_clearance
Gaps are shown in millimetres. The 5-DOF (no roll joint) is drawn as a horizontal line at its roll-0 value.
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read(path):
    xs, ys = [], []
    for row in csv.DictReader(open(os.path.expanduser(path))):
        if row["min_valid_gap_m"] != "":
            xs.append(float(row["roll_deg"]))
            ys.append(1000.0 * float(row["min_valid_gap_m"]))
    return xs, ys


ap = argparse.ArgumentParser()
ap.add_argument("--six", required=True)
ap.add_argument("--five")
ap.add_argument("--out", default="fig_clearance")
a = ap.parse_args()

plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7,
                     "xtick.labelsize": 7, "ytick.labelsize": 7})
fig, ax = plt.subplots(figsize=(3.5, 2.5))
x6, y6 = read(a.six)
ax.plot(x6, y6, "o-", ms=3, lw=1.2, label="6-DOF (wrist roll)")
if a.five:
    _, y5 = read(a.five)
    if y5:
        ax.axhline(y5[0], color="tab:red", ls="--", lw=1.2, label=f"5-DOF (no roll): {y5[0]:.0f} mm")
ax.set_xlabel("Wrist roll (deg)")
ax.set_ylabel("Smallest passable gap (mm)")
ax.set_xlim(min(x6) - 5, max(x6) + 5)
ax.grid(alpha=0.3)
ax.legend(loc="lower center")
fig.tight_layout()
fig.savefig(a.out + ".png", dpi=300)
fig.savefig(a.out + ".pdf")
print("wrote", a.out + ".png and .pdf")
