#!/usr/bin/env python3
"""
Classify every planning request in a saved move_group log (ros2 launch ... 2>&1 | tee file.log).
Runs are separated by gaps > GAP seconds between requests.

  python3 classify_failures.py ~/runs_v2/move_group_6dof.log
Per run it prints: requests, successes, failures by type, and which body pairs collided.
Types:  GOAL_SAMPLING   = OMPL could not sample a valid goal state ("Unable to sample any valid states")
        PATH_COLLISION  = OMPL found a path but ValidateSolution rejected it (contact pairs listed)
        OTHER_FAILURE   = failed, neither of the above
"""
import re, sys, os, collections

GAP = 3.0
ts_re = re.compile(r'\[(\d{10}\.\d+)\]')
pair_re = re.compile(r"Found a contact between '([^']+)' .*? and '([^']+)'")

blocks, cur = [], None
for line in open(os.path.expanduser(sys.argv[1]), errors="ignore"):
    m = ts_re.search(line)
    t = float(m.group(1)) if m else None
    if "MoveGroupMoveAction: Received request" in line:
        cur = {"t": t, "lines": []}
        blocks.append(cur)
    elif cur is not None:
        cur["lines"].append(line)

def classify(b):
    text = "".join(b["lines"])
    pairs = set()
    for a, c in pair_re.findall(text):
        pairs.add(tuple(sorted((a, c))))
    if "DisplayMotionPath" in text and "failed with error code" not in text:
        return "SUCCESS", pairs
    if "Unable to sample any valid states" in text or "Insufficient states in sampleable goal" in text:
        return "GOAL_SAMPLING", pairs
    if "ValidateSolution' failed" in text:
        return "PATH_COLLISION", pairs
    return "OTHER_FAILURE", pairs

runs, last = [], None
for b in blocks:
    if last is None or b["t"] - last > GAP:
        runs.append([])
    runs[-1].append(b)
    last = b["t"]

for i, r in enumerate(runs):
    c, pc = collections.Counter(), collections.Counter()
    for b in r:
        k, pairs = classify(b)
        c[k] += 1
        if k == "PATH_COLLISION":
            for p in pairs: pc[p] += 1
    print(f"run {i}: t={r[0]['t']:.0f}  requests={len(r)}  {dict(c)}")
    for p, n in pc.most_common(5):
        print(f"      path collision pair {p}: in {n} trials")