[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/kf8KOLnX)
# Exercise 8: Feedback-driven pick & place (no sleep, with timeouts & recovery)

Gate each step on `/joint_states` convergence (|err| < ε for N cycles) instead of sleep.

Add soft limits, timeout, and recovery: re-plan once, then fail safe to HOME.

## Do NOT use

- `time.sleep()` to advance steps
- MoveIt
- Actions
- Behavior trees
- External SM frameworks

## Deliverable

Same pick-place as Ex7 but deterministic and fault-tolerant.

## Required logs

Print per step:
```
[3/8] pose=pick  OK   reason=converged  duration=1.82s
[4/8] grip=close OK   reason=converged  duration=0.76s
```

…and a final summary table with step index, type, target, OK/FAIL, reason, duration.

## Concepts Covered

- Feedback loops
- Tolerances
- Watchdogs
- Retries

## What it is

Build a closed-loop controller for the OpenManipulator arm that executes a sequence of poses (home → ready → pick → close → place → open → home) based on real joint feedback from `/joint_states`, instead of relying on fixed `sleep()` delays.

Each step is only accepted when every commanded joint angle is within a small tolerance (ε = 0.02 rad) of the target for N consecutive control cycles (e.g., 10 cycles at 20 Hz).

If the robot doesn't converge in time, the controller retries once and, if that fails, recovers to HOME safely.

## Why this is important

Robots in real environments experience latency, friction, and disturbances.

Hard-coded delays make systems unpredictable and brittle — sometimes too short, sometimes wasting time.

A feedback-driven loop ensures that:

- Every motion completes only when physically achieved, not when time expires.
- Timeouts and retries handle stalls or controller hiccups.
- Fail-safe recovery prevents runaway arms by returning to a safe pose.
- You practice the principles of watchdogs, tolerance windows, and state machines.

## Concepts reinforced

| Concept | Description |
|---------|-------------|
| Feedback loop | The node reads actual joint states and compares them to desired targets. |
| Tolerance | Defines acceptable error margin before a motion is considered stable. |
| Watchdog / timeout | Detects when a command fails to reach its target within allowed time. |
| Retries | Gives one more chance before failing hard. |
| Recovery | Automatic transition to a safe, predefined posture (HOME). |
| Determinism | The behavior no longer depends on arbitrary sleeps — it's governed by real robot feedback. |

## What you learn

- How to use `/joint_states` feedback effectively in control loops.
- How to validate robot motion rather than assuming it succeeded.
- How to implement finite-state machines (step progression logic).
- How to design fault-tolerant control code (timeouts, retries, recovery).
- The difference between open-loop (command → sleep) and closed-loop (command → observe → confirm).
