# Simulation results (CrazySim)

What the simulator data shows, how far it can be trusted, and what is still
missing. Numbers come from `data/raw/*_sim_*`. To reproduce them:

```bash
python -m analysis.summarize_second_order_sim      # batch 2: second order, Fitts
python -m analysis.summarize_sim_runs              # batch 1: 2 Nov runs
python -m analysis.summarize_sim_runs --since 0    # batch 1: including 28 Oct
```

---

## 1. Summary

| Question | Simulator answer | Confidence |
|---|---|---|
| 1. Does neighbour surprise keep a leaderless swarm spaced? | **Yes.** Seen in both batches, and it matches the real drones | Medium to good |
| 2. Is second order better than first order? | **Two drones: yes**, faster and smoother at the same speed limit. **Swarm: smoother yes; faster probably**, but the speed limits differ | Medium |
| 3. Does second order follow Fitts' law? | **Not shown.** Arrival time grows with distance, but a straight line fits as well as the log law | Low |
| 4. Does the adaptive gain beat a fixed gain? | **Mixed.** Clearly better when too far; for too close, batch 1 says yes and batch 2 says no | Low to medium |
| H1. Does convoy gating reduce bunching? | **Not supported** in batch 2, and not robust in batch 1 | Low |
| H2. Is there a useful range of N? | **N = 0.05 freezes the convoy**, repeated in both batches. N ≥ 0.1 does not | Good |
| H3. Does the formation controller converge? | **Yes, in the one run we have** | Low (n = 1) |

**The biggest limitation:** batch 2 ran on an overloaded setup, and 63 of its 81
planned runs failed. See §2.

---

## 2. The data and its quality

| | Batch 1 | Batch 2 |
|---|---|---|
| Dates | 28 Oct (partial), 2 Nov | 4–5 Nov |
| Code | `86f6f60` | `3106f2f` |
| Experiments | two-drone first order, `target_swarm`, convoy | all, including second order and Fitts sweeps |
| Runs saved | 41 (40 finished) | 67 (46 finished) |
| Longest gap between control steps (median run) | **0.11 s** | **0.35 s** (worst 1.39 s) |
| Start layout of `target_swarm` | line along y | staggered, same as real flights |

Noise std is 0.03 (m, m/s) in both.

**Why batch 2 failed so often.** `sim_batch.log` lists **63 of 81** runs of its last
pass as failed, almost all with `ABORT: no fresh data` after telemetry stalled for
0.8–1.3 s. Many runs stopped at about 20 s. One run aborted with a position below
the floor. This is the simulator setup, not a controller problem: the batch ran
under WSL from `/mnt/d`, and batch 1 had none of these stalls. The exact cause of
the stalls has not been confirmed.

**What this means for the numbers:**
- 28 of the 46 finished runs have no step gap over 0.5 s ("clean"). The rest had a
  1 s stall during the run. Batch 2 tables use all finished runs; the clean subset
  gave similar results, but often has n = 1.
- Aborted runs are excluded, so some conditions have only 1 or 2 runs.
- Several runs share seed 101 because of repeats, so n does not mean n different
  seeds.
- `analysis/figures/*_sim.png` mix both batches and include the stalled runs. Use
  the tables here for numbers. The Fitts panel of the pair figure uses
  `settling_time`, not the `hold_time` used here.

---

## 3. Question 1: neighbour surprise in the swarm

| Metric | Batch 2 `goal_only` (n = 2) | Batch 2 `surprise` (n = 1) | Real `goal_only` (n = 1) | Real `surprise` (n = 1) |
|---|---|---|---|---|
| Final spacing error (m) | 0.229 | 0.047 | 0.251 | 0.102 |
| Safety push active | 37 % | 0 % | 61 % | 0 % |
| Smallest pair distance (m) | 0.265 | 0.496 | 0.224 | 0.418 |
| Arrival time (s) | 22.2 ± 5.5 | 25.1 | 15.5 | 21.2 |

Batch 1 with the same seed (103) agrees: spacing error 0.243 vs 0.014 m, safety push
59 % vs 1.8 %. With neighbour surprise the drones form an even triangle and do not
need the push. Without it they crowd, and only the push separates them. This is
the most reliable result: **it holds across two batches and on real drones.**

The cost is arrival time. It was about 35–40 % later on real drones and in batch 1, but
in batch 2 the difference is within the spread.

The second-order swarm shows the same pattern (mean spacing 0.54 m with neighbours,
0.42 m without). Ignore `final_spacing_error` = 0.078 m for `goal_only_second_order`:
that run keeps bouncing between about 0.33 and 0.6 m, and the metric reads only the
last sample.

---

## 4. Question 2: first order vs second order

This is the fair comparison the real data lacks: same batch, same noise.

**Two drones** (speed limit 0.10 m/s for all). `hold_time` = first time the gap stays
within ±8 cm of 0.60 m for 2 s.

| Scenario | Controller | Runs | Hold time (s) | RMS error (m) | Command jerk (m/s²) |
|---|---|---|---|---|---|
| Too far | fixed | 3 | 5.0 ± 0.6 | 0.091 | 0.134 |
| | adaptive | 3 | 2.9 ± 0.8 | 0.055 | 0.183 |
| | **second order** | 2 | **2.7 ± 0.1** | 0.054 | **0.081** |
| | second order adaptive | 3 | 2.4 ± 0.4 | 0.049 | 0.104 |
| Too close | fixed | 2 | 6.2 ± 0.3 | 0.068 | 0.160 |
| | adaptive | 2 | 6.8 ± 2.1 | 0.065 | 0.188 |
| | **second order** | 2 | **1.8 ± 0.2** | 0.040 | **0.101** |
| | second order adaptive | 3 | 1.7 ± 0.3 | 0.039 | 0.115 |

Second order is about as fast as adaptive when too far and over 3× faster when too
close, with about 45–55 % less jerk than adaptive. Adding the adaptive gain to second
order changes little and raises jerk slightly.

**Three drones** (default target, same staggered start, about 1.33 m from the target).

| Metric | First-order `surprise` (n = 1) | Second order (n = 3) |
|---|---|---|
| Arrival time (s) | 25.1 | **11.8 ± 0.2** |
| Command jerk (m/s²) | 0.95 | **0.08** |
| Smallest pair distance (m) | 0.496 | 0.474 ± 0.034 |
| Final mean spacing (m) | 0.58 | 0.54 |
| **Speed limit (m/s)** | **0.10** | **0.15** |

- **Smoothness is a strong result:** about 12× lower jerk, and the same direction in
  the baselines (0.46 vs 0.18). A speed limit cannot explain this. It makes sense:
  first order turns noisy positions directly into velocity, and second order
  smooths them through acceleration.
- **Speed: probably faster, but not proven.** It arrived about 2.1× sooner, while the
  speed limit alone allows up to 1.5×. That leaves roughly 1.4×, from one first-order
  run. One run at equal speed limits would settle it.
- **Cost:** spacing ends slightly tighter (0.54 vs 0.58 m), because of the softer
  neighbour spring.

---

## 5. Question 3: Fitts' law

Both sweeps were run, but **neither confirms the law**. The fits use W = 0.20 m.

| Sweep | Fit | Equation | R² | n |
|---|---|---|---|---|
| Swarm, all distances | log | MT = 0.59 + 3.83 · ID | 0.885 | 9 |
| | linear | MT = 4.57 + 5.51 · D | **0.929** | 9 |
| Swarm, D ≤ 0.8 m | log | MT = 4.36 + 1.69 · ID | 0.288 | 4 |
| | linear | MT = 5.62 + 3.46 · D | 0.334 | 4 |
| Pair | log | MT = −1.62 + 2.56 · ID | **0.984** | 6 |
| | linear | MT = −0.15 + 11.79 · D | 0.952 | 6 |

- **Swarm:** beyond D ≈ 0.8 m a straight line fits better. That is what a speed limit
  produces, so these runs test the limit, not the controller. Below 0.8 m there are
  too few runs and the fit is poor, and arrival stays at 6–9 s.
- **Pair:** the log fit is slightly better, but over a narrow range (ID 0.9–2.3), with
  6 runs, at the 0.10 m/s limit.

To test this properly, raise the limit so the drones never reach it, and use a wider
range. That is safe in simulation; see §8.

---

## 6. Question 4: adaptive vs fixed gain

| Scenario | Batch 1, hold time (s) | Batch 2, hold time (s) |
|---|---|---|
| Too far, fixed → adaptive | 5.7 → 2.9 | 5.0 → 2.9 |
| Too close, fixed → adaptive | 4.6 → 2.4 | 6.2 → 6.8 |

- **Too far: consistent.** Adaptive is about 2 s faster in both batches.
- **Too close: inconsistent.** Batch 2 shows no benefit, though with only 2 runs each
  and stall effects. The earlier "adaptive won all 6 pairs" holds for batch 1 only.
- Adaptive also raises jerk by about 20–35 %.
- Any benefit shows that a stiffer correction helps, not surprise as such, because
  surprise here depends only on the current error (EXPERIMENTS.md §1).

---

## 7. Convoy and formation

Convoy, 60 s runs, target gap 1.0 m. Batch 1 is the 2 Nov runs (n = 3 each). Batch 2
has n = 1 per condition, except N = 0.05 (n = 2).

| Condition | Batch 1 overshoot (m) | Batch 2 overshoot (m) | Final gap error (m) | Speed factor < 0.1 |
|---|---|---|---|---|
| E2 baseline | 0.116 ± 0.066 | 0.087 | 0.05 | 0 % |
| E1, N = 0.05 | 0.000 | 0.02 | **1.25–1.34** | **23–27 %** |
| E1, N = 0.1 | 0.084 ± 0.023 | 0.076 | 0.05 | under 1 % |
| E1, N = 0.2 | 0.057 ± 0.008 | 0.093 | 0.02–0.05 | 0 % |
| E1, N = 0.4 | 0.084 ± 0.038 | 0.086 | 0.02–0.05 | 0 % |

- **H2:** N = 0.05 is not a good result despite low overshoot. The followers freeze and
  CF2 ends over 3 m behind, in both batches. Velocity noise of 0.03 m/s already looks
  "surprising" at this N.
- **H1:** not supported. The N = 0.2 improvement in the 2 Nov runs disappears with the
  28 Oct runs (0.185 m for both) and in batch 2 (0.093 vs 0.087 m).
- The scenario is mild: neither version needed the safety push.

Formation E3 (one finished run, batch 2): gaps converged in 10.2 s, final gap error
0.016 m, U = 0.0005. This supports H3, but it is one run.

---

## 8. What to run next

**First, fix the stalls.** On the server: copy the repo into the Linux file system
(for example `~/Drone-Surprisal`, not `/mnt/d`), close other heavy programs, and
install matplotlib in the venv (6 analysis steps failed without it):

```bash
pip install -r requirements.txt
```

Then use one short check before any full batch. Look for no `no fresh data` errors:

```bash
REPEATS=1 ONLY=target_swarm ./scripts/run_all_sim.sh 2>&1 | tee sim_check.log
```

**Then, in priority order:**

| Priority | Command | Why |
|---|---|---|
| 1 | `N_DRONES=3 ./scripts/run_experiment.sh second_order_swarm --mode sim --controller second_order --set max_horizontal_speed=0.10 --noise 0.03 --seed 101` (seeds 101–103) | Equal speed limit settles question 2 for the swarm |
| 1 | `ONLY=target_swarm ./scripts/run_all_sim.sh 2>&1 \| tee sim_target.log` | Only one `surprise` run in batch 2 |
| 2 | `N_DRONES=3 ./scripts/run_experiment.sh second_order_swarm --mode sim --controller second_order --target-distance 0.8 --set max_horizontal_speed=0.40 --noise 0.03 --seed 101` (D = 0.4, 0.6, 0.8, 1.2, 1.5, 2.0, seeds 101–103) | Fitts sweep without the speed limit |
| 3 | `ONLY=goal_navigation ./scripts/run_all_sim.sh 2>&1 \| tee sim_pair1.log` | Settles the too-close adaptive question |
| 3 | `ONLY=formation ./scripts/run_all_sim.sh 2>&1 \| tee sim_form.log` | Formation has one run |

**Do not compare with real drones:** jerk (sim noise is independent at every step, so
sim jerk is higher) or absolute arrival times.

**Metric fixes to make:** use mean spacing over the last 5 s for
`final_spacing_error`, and `hold_time` in the pair Fitts plot.
