#!/usr/bin/env python3
"""
Read-only re-analysis of the corridor width-sweep databases (planning_5dof_w*.db / planning_6dof_w*.db).

- Never modifies the databases (opened with mode=ro).
- Flags 5-DOF "placeholder" goals (all-zero joint targets) and excludes them from 5-DOF summaries.
- Treats the GOAL as the unit of analysis (cluster bootstrap over goals), as reviewer comment 2 asks.
- Compares 5-DOF vs 6-DOF only on the roll-0 goal (reviewer comment 1). That comparison is PROVISIONAL until
  you confirm both goals reach the same Cartesian pose (forward kinematics check).
- Reports failure timing, to show whether failures are fast rejections or planner timeouts.

Usage:
    python3 reanalysis.py --sweep ~/omx_vnc_backup/width_sweep_70trials_20260222_144555 --out ~/reanalysis_out
Needs: python3-numpy python3-scipy (python3-matplotlib optional, for figures).
"""
import argparse, ast, csv, datetime, glob, json, os, re, sqlite3, sys
from pathlib import Path
import numpy as np
from scipy import stats

FAST_FAIL_S = 0.1  # failures faster than this are "instant" (not a search that ran out of time)


def connect_ro(path):
    return sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True)


def parse_cfg(text):
    for fn in (json.loads, ast.literal_eval):
        try:
            v = fn(text)
            if isinstance(v, dict):
                v = list(v.values())
            return [float(x) for x in v]
        except Exception:
            pass
    return []


def clopper_pearson(s, n, alpha=0.05):
    if n == 0:
        return (np.nan, np.nan)
    lo = 0.0 if s == 0 else stats.beta.ppf(alpha / 2, s, n - s + 1)
    hi = 1.0 if s == n else stats.beta.ppf(1 - alpha / 2, s + 1, n - s)
    return (float(lo), float(hi))


def cluster_bootstrap(rates, B=10000, seed=0):
    """Mean of per-goal rates with a percentile CI from resampling GOALS (goal = unit)."""
    rates = np.asarray(rates, float)
    if len(rates) == 0:
        return (np.nan, np.nan, np.nan)
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(rates), size=(B, len(rates)))
    means = rates[idx].mean(axis=1)
    return (float(rates.mean()), float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5)))


def load_file(path):
    m = re.match(r"planning_(5dof|6dof)_w([0-9.]+)\.db$", os.path.basename(path))
    if not m:
        return None
    cfg_type, width = m.group(1), float(m.group(2))
    con = connect_ro(path)
    cols = {r[1] for r in con.execute("pragma table_info(planning_results)")}
    need = {"goal_id", "success", "planning_time", "error_code", "goal_config"}
    if not need <= cols:
        print(f"  skip {path}: missing columns {need - cols}")
        return None
    rows = con.execute(
        "select goal_id, success, planning_time, error_code, goal_config from planning_results order by id").fetchall()
    con.close()
    goals = {}
    for gid, succ, t, ec, cfg in rows:
        g = goals.setdefault(gid, dict(n=0, s=0, cfg=parse_cfg(cfg), t_ok=[], t_fail=[], codes={}))
        g["n"] += 1
        g["s"] += int(succ)
        (g["t_ok"] if succ else g["t_fail"]).append(float(t))
        if not succ:
            g["codes"][ec] = g["codes"].get(ec, 0) + 1
    for gid, g in goals.items():
        g["placeholder"] = bool(g["cfg"]) and all(abs(x) < 1e-12 for x in g["cfg"])
        if cfg_type == "6dof" and len(g["cfg"]) > 4:
            g["roll"] = int(round(np.degrees(g["cfg"][4])))
        else:
            g["roll"] = None
    return dict(path=path, cfg=cfg_type, width=width, rows=len(rows), goals=goals,
                mtime=datetime.datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M"))


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def fmt(x, nd=3):
    return "  n/a" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", default=os.path.expanduser("~/omx_vnc_backup/width_sweep_70trials_20260222_144555"))
    ap.add_argument("--out", default=os.path.expanduser("~/reanalysis_out"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    files = [load_file(p) for p in sorted(glob.glob(os.path.join(a.sweep, "planning_*_w*.db")))]
    files = [f for f in files if f]
    if not files:
        sys.exit("No planning_*_w*.db files found in " + a.sweep)
    D = {(f["cfg"], f["width"]): f for f in files}
    widths = sorted({f["width"] for f in files})

    # ---- T0 inventory -------------------------------------------------------------------
    print("\n=== T0  INVENTORY (what exists) ===")
    inv = []
    for f in sorted(files, key=lambda x: (x["cfg"], x["width"])):
        ph = sum(g["placeholder"] for g in f["goals"].values())
        inv.append([os.path.basename(f["path"]), f["cfg"], f["width"], f["rows"], len(f["goals"]), ph, f["mtime"]])
        print(f"  {inv[-1][0]:28s} rows={f['rows']:4d} goals={len(f['goals'])} placeholder_goals={ph}  modified {f['mtime']}")
    for c in ("5dof", "6dof"):
        missing = [w for w in widths if (c, w) not in D]
        if missing:
            print(f"  NOTE: no {c} file for width(s) {missing}  -> not run, or stored elsewhere")
    write_csv(f"{a.out}/T0_inventory.csv", ["file", "config", "width_m", "rows", "goals", "placeholder_goals", "modified"], inv)

    # ---- T1 per-goal --------------------------------------------------------------------
    print("\n=== T1  PER-GOAL RESULTS (Clopper-Pearson 95% CI) ===")
    t1 = []
    for f in sorted(files, key=lambda x: (x["cfg"], x["width"])):
        for gid, g in sorted(f["goals"].items()):
            lo, hi = clopper_pearson(g["s"], g["n"])
            t1.append([f["cfg"], f["width"], gid, g["roll"], g["n"], g["s"], g["s"] / g["n"], lo, hi,
                       "PLACEHOLDER" if g["placeholder"] else "",
                       np.mean(g["t_ok"]) if g["t_ok"] else "", np.mean(g["t_fail"]) if g["t_fail"] else ""])
            flag = "  <-- PLACEHOLDER (home pose), excluded" if g["placeholder"] else ""
            print(f"  {f['cfg']} w={f['width']:.2f} goal={gid} roll={g['roll']!s:>4} {g['s']:2d}/{g['n']:2d} "
                  f"[{fmt(lo,2)},{fmt(hi,2)}]{flag}")
    write_csv(f"{a.out}/T1_per_goal.csv",
              ["config", "width_m", "goal_id", "roll_deg", "n", "successes", "rate", "ci_lo", "ci_hi", "flag",
               "mean_time_success_s", "mean_time_fail_s"], t1)

    # ---- T2 goal-level summary ----------------------------------------------------------
    print("\n=== T2  SUMMARY PER WIDTH (valid goals only; goal = unit; cluster bootstrap over goals) ===")
    t2, series = [], {}
    for f in sorted(files, key=lambda x: (x["cfg"], x["width"])):
        valid = [g for g in f["goals"].values() if not g["placeholder"]]
        rates = [g["s"] / g["n"] for g in valid]
        m, lo, hi = cluster_bootstrap(rates)
        t2.append([f["cfg"], f["width"], len(valid), sum(g["n"] for g in valid), sum(g["s"] for g in valid), m, lo, hi])
        series[(f["cfg"], f["width"])] = (m, lo, hi, len(valid))
        warn = "  (ONE goal only: bootstrap over goals not meaningful, see T1 CI)" if len(valid) < 2 else ""
        print(f"  {f['cfg']} w={f['width']:.2f} valid_goals={len(valid)} mean_goal_rate={fmt(m)} "
              f"[{fmt(lo)},{fmt(hi)}]{warn}")
    write_csv(f"{a.out}/T2_goal_level_summary.csv",
              ["config", "width_m", "valid_goals", "trials", "successes", "mean_goal_rate", "boot_lo", "boot_hi"], t2)

    # ---- T3 common-target comparison ----------------------------------------------------
    print("\n=== T3  ROLL-0 COMPARISON, 5-DOF real goal vs 6-DOF roll-0 goal  (PROVISIONAL, small n) ===")
    print("  Provisional until you confirm both goals are the same Cartesian pose (forward kinematics).")
    t3 = []
    for w in widths:
        f5, f6 = D.get(("5dof", w)), D.get(("6dof", w))
        if not (f5 and f6):
            print(f"  w={w:.2f}  missing a config -> no comparison")
            continue
        real5 = [g for g in f5["goals"].values() if not g["placeholder"]]
        g6 = [g for g in f6["goals"].values() if g["roll"] == 0]
        if len(real5) != 1 or len(g6) != 1:
            print(f"  w={w:.2f}  could not identify exactly one real 5-DOF goal and one 6-DOF roll-0 goal")
            continue
        a5, a6 = real5[0], g6[0]
        p = stats.fisher_exact([[a5["s"], a5["n"] - a5["s"]], [a6["s"], a6["n"] - a6["s"]]])[1]
        t3.append([w, a5["s"], a5["n"], a6["s"], a6["n"], p])
        print(f"  w={w:.2f}  5-DOF {a5['s']}/{a5['n']}   6-DOF(roll0) {a6['s']}/{a6['n']}   Fisher p={p:.3f}")
    write_csv(f"{a.out}/T3_roll0_comparison.csv",
              ["width_m", "5dof_succ", "5dof_n", "6dof_roll0_succ", "6dof_roll0_n", "fisher_p"], t3)

    # ---- T4 roll trend (6-DOF only) -----------------------------------------------------
    print("\n=== T4  WITHIN-6-DOF ROLL EFFECT (Spearman across the 7 goals; n=7 is small) ===")
    t4 = []
    for w in widths:
        f6 = D.get(("6dof", w))
        if not f6:
            continue
        pts = sorted((g["roll"], g["s"] / g["n"]) for g in f6["goals"].values() if g["roll"] is not None)
        x, y = [p[0] for p in pts], [p[1] for p in pts]
        if len(set(y)) < 2:
            rho, p = np.nan, np.nan
        else:
            rho, p = stats.spearmanr(x, y)
        t4.append([w, rho, p])
        print(f"  w={w:.2f}  rates by roll {[round(v, 2) for v in y]}  rho={fmt(rho,2)} p={fmt(p,3)}")
    write_csv(f"{a.out}/T4_roll_trend.csv", ["width_m", "spearman_rho", "p"], t4)

    # ---- T5 failure timing --------------------------------------------------------------
    print(f"\n=== T5  FAILURE TIMING (instant = under {FAST_FAIL_S}s; timeouts would be ~10s) ===")
    t5 = []
    for f in sorted(files, key=lambda x: (x["cfg"], x["width"])):
        tf = [t for g in f["goals"].values() for t in g["t_fail"]]
        codes = {}
        for g in f["goals"].values():
            for c, k in g["codes"].items():
                codes[c] = codes.get(c, 0) + k
        if not tf:
            continue
        fast = sum(t < FAST_FAIL_S for t in tf)
        t5.append([f["cfg"], f["width"], len(tf), fast, float(np.median(tf)), float(max(tf)), json.dumps(codes)])
        print(f"  {f['cfg']} w={f['width']:.2f} failures={len(tf):3d} instant={fast:3d} "
              f"median={np.median(tf):.2f}s max={max(tf):.2f}s codes={codes}")
    write_csv(f"{a.out}/T5_failure_timing.csv",
              ["config", "width_m", "failures", "instant_failures", "median_s", "max_s", "error_codes"], t5)

    # ---- Figures ------------------------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams.update({"font.size": 13, "axes.labelsize": 14, "axes.titlesize": 14})

        # Fig 1: 6-DOF heatmap roll x width
        rolls = sorted({g["roll"] for f in files if f["cfg"] == "6dof" for g in f["goals"].values()
                        if g["roll"] is not None})
        w6 = [w for w in widths if ("6dof", w) in D]
        if rolls and w6:
            M = np.full((len(rolls), len(w6)), np.nan)
            T = [[""] * len(w6) for _ in rolls]
            for j, w in enumerate(w6):
                for g in D[("6dof", w)]["goals"].values():
                    if g["roll"] in rolls:
                        i = rolls.index(g["roll"])
                        M[i, j] = g["s"] / g["n"]
                        T[i][j] = f"{g['s']}/{g['n']}"
            fig, ax = plt.subplots(figsize=(7, 4.5))
            im = ax.imshow(M, vmin=0, vmax=1, cmap="viridis", aspect="auto", origin="lower")
            ax.set_xticks(range(len(w6)))
            ax.set_xticklabels([f"{w:.2f}" for w in w6])
            ax.set_yticks(range(len(rolls)))
            ax.set_yticklabels([f"{r}°" for r in rolls])
            ax.set_xlabel("Corridor width (m)")
            ax.set_ylabel("Wrist roll (6-DOF)")
            for i in range(len(rolls)):
                for j in range(len(w6)):
                    v = M[i, j]
                    ax.text(j, i, T[i][j], ha="center", va="center",
                            color="white" if (not np.isnan(v) and v < 0.6) else "black", fontsize=11)
            fig.colorbar(im, ax=ax, label="Planning success rate")
            fig.tight_layout()
            fig.savefig(f"{a.out}/fig_roll_vs_width_6dof.png", dpi=300)
            fig.savefig(f"{a.out}/fig_roll_vs_width_6dof.pdf")
            plt.close(fig)

        # Fig 2: success vs width
        fig, ax = plt.subplots(figsize=(7, 4.5))
        def line(label, xs, ys, los, his, marker):
            err = [np.array(ys) - np.array(los), np.array(his) - np.array(ys)]
            ax.errorbar(xs, ys, yerr=err, marker=marker, capsize=4, label=label)
        xs, ys, lo_, hi_ = [], [], [], []
        for w in w6:
            m, lo, hi, _ = series[("6dof", w)]
            xs.append(w); ys.append(m); lo_.append(lo); hi_.append(hi)
        if xs:
            line("6-DOF, all rolls (mean over goals, bootstrap 95% CI)", xs, ys, lo_, hi_, "o")
        xs, ys, lo_, hi_ = [], [], [], []
        for w in w6:
            g = [g for g in D[("6dof", w)]["goals"].values() if g["roll"] == 0]
            if g:
                lo, hi = clopper_pearson(g[0]["s"], g[0]["n"])
                xs.append(w); ys.append(g[0]["s"] / g[0]["n"]); lo_.append(lo); hi_.append(hi)
        if xs:
            line("6-DOF, roll 0 goal (n=10, exact 95% CI)", xs, ys, lo_, hi_, "s")
        xs, ys, lo_, hi_ = [], [], [], []
        for w in widths:
            f5 = D.get(("5dof", w))
            if f5:
                real = [g for g in f5["goals"].values() if not g["placeholder"]]
                if len(real) == 1:
                    lo, hi = clopper_pearson(real[0]["s"], real[0]["n"])
                    xs.append(w); ys.append(real[0]["s"] / real[0]["n"]); lo_.append(lo); hi_.append(hi)
        if xs:
            line("5-DOF, real goal (n=10, exact 95% CI)", xs, ys, lo_, hi_, "^")
        ax.set_xlabel("Corridor width (m)")
        ax.set_ylabel("Planning success rate")
        ax.set_ylim(-0.02, 1.05)
        ax.legend(fontsize=10, loc="lower right")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(f"{a.out}/fig_success_vs_width.png", dpi=300)
        fig.savefig(f"{a.out}/fig_success_vs_width.pdf")
        plt.close(fig)
        print(f"\nFigures written to {a.out}")
    except ImportError:
        print("\n(matplotlib not installed: skipped figures. sudo apt install python3-matplotlib)")

    print(f"CSV tables written to {a.out}")


if __name__ == "__main__":
    main()
