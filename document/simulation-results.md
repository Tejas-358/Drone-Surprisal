# Simulation results (CrazySim)

What the simulator data shows, how far it can be trusted, and what is still
missing. Numbers come from `data/raw/*_sim_*`. To reproduce them:

```bash
python -m analysis.summarize_sim_runs              # 2 Nov batch (used below)
python -m analysis.summarize_sim_runs --since 0    # all runs, including 28 Oct
```

---

## 1. Summary

| Question | Simulator answer | Confidence |
|---|---|---|
| Does neighbour surprise keep a leaderless swarm spaced? | **Yes, consistent with the real drones** | Medium: only one `goal_only` run |
| Does the adaptive gain correct spacing faster than a fixed gain? | **Yes, in all 6 paired runs**, about 2 s faster | Medium: n = 3 per condition, and see §4 |
| Does convoy surprise gating reduce bunching after the leader stops? (H1) | **Not shown.** Better in the 2 Nov batch, gone when 28 Oct is included | Low |
| Is there a useful range of N? (H2) | **Partly.** N = 0.05 clearly freezes the convoy; N ≥ 0.1 does not | Good for the freezing, low for the best N |
| Is second order faster and smoother? Does it follow Fitts' law? | **No sim data at all** | None |
| Does the formation controller converge? (H3) | **No sim data at all** | None |

**The batch was run on old code** (commit `86f6f60`), before the second-order
experiments existed. See §6 for what to run next.

---

## 2. What data we have

41 simulator runs: 40 finished, 1 aborted. Noise std 0.03 (m, m/s) on what the
controller sees. Runs that stopped early are excluded from all results.

| Experiment | Condition | 2 Nov batch (seeds 101–103) | 28 Oct batch (seed 101 mostly) |
|---|---|---|---|
| Two drones | fixed / adaptive × too far / too close | 3 each | 1 each, plus 1 aborted fixed run (seed 102) |
| Three drones | `surprise` | 2 (seeds 101, 103) | 1 |
| | `goal_only` | **1** (seed 103) | 0 |
| Convoy | baseline | 3 | 1 |
| | surprise, N = 0.05, 0.1, 0.2, 0.4 | 3 each | 1 each |
| Formation | | **0** | 0 |
| Second-order pair / swarm, Fitts sweeps | | **0** | 0 |

**Why the 2 Nov batch is used as the main data.** It is the complete repeat set.
The 28 Oct batch was partial and has unusually large errors, for example a
convoy gap of 0.43 m and a leader ending 0.5 m short of its stop point. The cause
is unknown (the batch log was not kept), so this is a choice, not a proven
fault. §5 shows where it changes the conclusion.

**Limits of these numbers:**
- Three runs per condition give a rough mean ± std, not statistical significance.
- Seed 101 was used in both batches, so those runs share a noise seed.
- Some starts missed their target: fixed seed 101 started 0.89 m (too far) and
  0.56 m (too close) instead of 0.80 and 0.45. `plot_goal_navigation_runs` skips
  the 0.56 m run because it started within 10 cm of the target; the tables here
  keep it.

---

## 3. Three drones to a target

Same seed (103), so these two runs are directly comparable.

| Metric | `goal_only` | `surprise` |
|---|---|---|
| Arrival time (s) | 19.8 | 28.0 |
| Final spacing error (m) | 0.243 | 0.014 |
| Smallest pair distance (m) | 0.28 | 0.39 |
| Safety push active | 58.8 % of steps | 1.8 % |
| Final total surprise | 19.1 | 2.8 |

Without neighbour surprise the drones crowd at the target and only the safety
push separates them. With it, they form an even triangle and rarely need the
push. The cost is arrival time: about 40 % slower.

The second `surprise` run in this batch (seed 101) never met the arrival rule
(centre within 0.10 m; it ended 0.14 m away), though its spacing error was small
(0.069 m).

**Agreement with the real drones**

| Metric | Sim `goal_only` | Real `goal_only` | Sim `surprise` | Real `surprise` |
|---|---|---|---|---|
| Final spacing error (m) | 0.243 | 0.251 | 0.014 | 0.102 |
| Safety push active | 59 % | 61 % | 1.8 % | 0 % |
| Smallest pair distance (m) | 0.28 | 0.22 | 0.39 | 0.42 |
| Arrival time (s) | 19.8 | 15.5 | 28.0 | 21.2 |

The pattern matches closely, which is the strongest finding here. Note that
these sim runs used the older start layout (three drones in a line along y),
while the real runs used the staggered one, so arrival times differ. Each cell
is a single run.

---

## 4. Two drones: fixed vs adaptive gain

`hold_time` = first time the gap stays within ±8 cm of 0.60 m for 2 s. It replaces
`settling_time`, which noise makes unreliable.

| Scenario | Metric | Fixed | Adaptive |
|---|---|---|---|
| Too far | hold time (s) | 5.7 ± 1.3 (one run never held) | **2.9 ± 0.2** |
| | RMS error (m) | 0.113 ± 0.034 | **0.059 ± 0.009** |
| | closest distance (m) | 0.596 ± 0.062 | 0.575 ± 0.025 |
| Too close | hold time (s) | 4.6 ± 0.3 | **2.4 ± 0.5** |
| | RMS error (m) | 0.069 ± 0.008 | **0.045 ± 0.001** |
| | closest distance (m) | 0.424 ± 0.029 | 0.449 ± 0.005 |

| Seed | Too far: fixed → adaptive hold (s) | Too close: fixed → adaptive hold (s) |
|---|---|---|
| 101 | 6.9 → 3.1 | 5.0 → 1.8 |
| 102 | never → 2.9 | 4.5 → 2.5 |
| 103 | 4.4 → 2.6 | 4.3 → 2.9 |

Adaptive won every pair and was less variable. Two cautions:
- **This shows a stiffer correction helps, not that surprise as such does.** Here
  surprise depends only on the current error (EXPERIMENTS.md §1). A controller
  with `kp` computed directly from error size would behave the same.
- In the 28 Oct batch, one adaptive too-close run was much worse (hold 7.5 s, RMS
  0.098 m), so adaptive is not always better. Fixed seed 102 (too far) never held
  the band; the reason has not been investigated.

Real-drone data is too thin to confirm this: one usable `fixed` run per scenario,
from a different session.

---

## 5. Convoy E1 vs E2 and the N sweep

60 s runs, target gap 1.0 m. Leader stops at about 32 s. 2 Nov batch, n = 3 each:

| Condition | Overshoot after stop (m) | Smallest gap (m) | Final gap error (m) | Final CF1–CF2 gap (m) | Speed factor < 0.1 |
|---|---|---|---|---|---|
| E2 baseline | 0.116 ± 0.066 | 0.884 | 0.049 | 1.03 | 0 % |
| E1, N = 0.05 | 0.000 ± 0.000 | 0.950 | **1.250** | **3.37** | **26 %** |
| E1, N = 0.1 | 0.084 ± 0.023 | 0.916 | 0.046 | 0.96 | 0.2 % |
| E1, N = 0.2 | 0.057 ± 0.008 | 0.943 | 0.023 | 1.00 | 0 % |
| E1, N = 0.4 | 0.084 ± 0.038 | 0.916 | 0.045 | 1.02 | 0 % |

- **N = 0.05 is not a good result, despite zero overshoot.** The followers freeze:
  CF2 ends over 3 m behind the leader. Noise of 0.03 m/s on each velocity already
  looks "surprising" at this N. This is the clearest support for H2.
- **N = 0.2 looks best in this batch**, with about half the baseline's overshoot.
- **That result is not robust.** Including the 28 Oct runs (n = 4), overshoot is
  0.185 ± 0.133 m for the baseline and 0.185 ± 0.221 m for N = 0.2, so no benefit
  remains. One N = 0.2 run reached a gap of 0.43 m. **H1 should be reported as
  unresolved.**
- Neither version came near the safety push (`repel_fraction` ≈ 0) in this batch,
  so this scenario is mild. A faster leader or smaller gap would test H1 harder.

---

## 6. What is missing, and what to run

| Missing | Needed for |
|---|---|
| All second-order runs | Research question 2 |
| Both Fitts sweeps | Research question 3 (real drones have no sweep data either) |
| `goal_only` swarm ×2 and a third `surprise` run | Research question 1 |
| Formation runs | H3 (can be dropped if it stays out of the report) |
| More convoy repeats, ideally a harder scenario | H1 |

**Before running:** commit and push the latest code, then on the server run
`git pull`. Check that `git log -1` shows the new commit, and save the log this
time.

```bash
ONLY=second_order_pair ./scripts/run_all_sim.sh 2>&1 | tee sim_pair.log
```
```bash
ONLY=second_order_swarm ./scripts/run_all_sim.sh 2>&1 | tee sim_swarm.log
```
```bash
ONLY=target_swarm ./scripts/run_all_sim.sh 2>&1 | tee sim_target.log
```
```bash
ONLY=formation ./scripts/run_all_sim.sh 2>&1 | tee sim_formation.log
```

The new `target_swarm` runs use the staggered start, so they match the real
flights better than the runs above. If formation fails again, its log will show
why.

**Do not compare with real drones:**
- **Jerk.** The sim noise is independent at every step, so sim jerk (0.14–0.82) is
  higher than real jerk for that reason alone.
- **Absolute arrival times**, until start layouts match.

---

## 7. Figures

| Figure | Shows |
|---|---|
| `analysis/figures/target_swarm_runs_sim.png` | Swarm paths, smallest spacing, total surprise |
| `analysis/figures/goal_navigation_runs_sim.png` | Two-drone distance and surprise over time |
| `analysis/figures/convoy_runs_sim.png` | Convoy gaps and speed factors |

These plot **all** finished runs, including 28 Oct, so they contain the outliers
discussed in §4 and §5.
