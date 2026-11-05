# Report plan (4 pages)

What we can write now, which figures to use, what data is still missing, and
which runs and analysis steps to do before writing the final version.

Numbers are from real Crazyflie runs in `data/raw/` unless marked as simulator
runs. Simulator results are in [simulation-results.md](simulation-results.md).

**Comparison rule:** compare controllers only with the same start distance, the
same speed limit and the same session. Results that break this rule are marked
below and must not be used as headline claims.

---

## 1. What data we have

| Experiment | Controller | Finished real runs | Aborted | Notes |
|---|---|---|---|---|
| Two drones, first order (`goal_navigation`) | `fixed` | 2 (19 Oct) | – | Only in `surprisal_goal_navigation_summary.csv`, **no JSON**, so the second-order analysis cannot see them |
| | `adaptive` | 6 (28 Oct) | – | 2 are skipped by the plot (started at target / 22 Lighthouse jumps). The 0.95 m run is `as_placed`, so the two plots count too-far runs differently (3 vs 2) |
| Two drones, second order (`second_order_pair`) | `second_order` | 4 (3 too_far, 1 too_close) | 7 | Aborts: empty battery, start-position flight lost Lighthouse tracking. Too-far runs started at 0.77, 1.00 and 1.39 m instead of 0.80 m |
| | `second_order_adaptive` | **0** | – | |
| | Fitts sweep (`--start-gap`) | **0** | – | |
| Three drones, first order (`target_swarm`) | `goal_only` | 1 | – | |
| | `surprise` | 1 | 3 | One aborted run was at 0.8 m spacing (39.9 s, 72 position jumps, never arrived): excluded |
| Three drones, second order (`second_order_swarm`) | `second_order` | 1 (after the damping fix) | 2 (before the fix) | Drones were not placed on the printed spots |
| | `goal_only_second_order` | **0** | – | |
| | Fitts sweep (`--target-distance`) | **0** | – | |
| Convoy E1/E2, formation E3 (real) | – | none in the current format | – | Old CSVs (12–13 Oct) have no JSON and are not used |
| CrazySim batch 1 (`86f6f60`) | two-drone first order, `target_swarm`, convoy | 40 | 1 | Clean runs. See `simulation-results.md` |
| CrazySim batch 2 (`3106f2f`) | all, with second order and Fitts sweeps | 46 | 21 saved, and the log lists 63 of 81 as failed | Telemetry stalls on the server. n = 1–3 per condition |
| Point-mass simulation (`surprisal_euler_lagrange_sim`) | fixed vs adaptive | figures in `data/simulation/euler_lagrange/` | – | Good illustration for the method section |

**Bottom line:** we have one or a few real runs per condition. That is enough to
show the idea works on real drones, but not for a mean ± std comparison. The
first-order vs second-order swarm comparison is **not like for like yet**, and
the Fitts' law claim is **not confirmed**: there is no real data, and the simulator sweep cannot separate the log law from a speed limit.

---

## 2. Results we can already state

### Two drones (real)

| Controller | Scenario | Runs | Settling time (s) | Overshoot (m) | RMS error (m) | Min distance (m) | Command jerk (m/s²) |
|---|---|---|---|---|---|---|---|
| adaptive (first order) | too far | 2 | 28.3 | 0.036 | 0.056 | 0.564 | 0.075 |
| adaptive (first order) | too close | 1 | 16.8 | 0.056 | 0.044 | 0.394 | 0.095 |
| second order | too far | 3 | 27.9 ± 2.8 | 0.069 ± 0.014 | 0.167 ± 0.084 | 0.531 | 0.068 ± 0.008 |
| second order | too close | 1 | 22.4 | 0.109 | 0.034 | 0.446 | 0.078 |

What to say:
- Both controllers bring the pair to 0.60 m and then hold it within about ±5 cm.
- Second order *may* give smoother commands (jerk about 10–20 % lower), but the runs are from different sessions and every two-drone run peaks at the 0.10 m/s limit. Do not state this as a result yet.
- Second-order overshoot is *larger* in these runs. Most of it comes from the drones wobbling straight after the old start-position flight (up to 0.3 m sideways in y), not from the controller. Say this openly and point to the `--placed` fix.
- "Settling time" is about 22–30 s for every controller, because Lighthouse jitter (±4–5 cm) keeps crossing the ±5 cm band. **Do not compare controllers with this metric** (see §5).
- RMS error for second order / too far is high because two runs started far off (1.00 and 1.39 m, intended 0.80 m). **Do not compare it with first order**, whose starts were 0.84–0.85 m.

### Three drones (real)

| Controller | Arrival time (s) | Final spacing error (m) | Safety push active | Min pair distance (m) | Command jerk (m/s²) |
|---|---|---|---|---|---|
| goal_only (first order) | 15.5 | 0.251 | 61 % of steps | 0.224 | 0.189 |
| surprise (first order) | 21.2 | 0.102 | 0 % | 0.418 | 0.503 |
| second order (surprise) | **7.2** | 0.108 | 1.2 % | 0.239 * | **0.071** |

\* happened at t = 0: the drones were placed 0.33 m apart instead of on the printed spots.

**The second-order row is not comparable with the two rows above.** It started
0.83 m from the target (first order: 1.35 m), had a 0.15 m/s speed limit (first
order: 0.10 m/s), was placed by hand, and was flown in a different session.
Allowing for distance, average approach speed is about 0.10 vs 0.06 m/s, or about
1.7× faster, and the speed limit alone is 1.5×.

What to say:
- Without neighbour surprise (`goal_only`), drones crowd at the target. Only the safety push keeps them apart, active 61 % of the time, with spacing about 0.3 m. With surprise they form a triangle with sides of about 0.55 m and almost never need the push. **This is the clearest result in the project.**
- **Do not claim "3× faster" or "7× smoother".** The speed gap is mostly the setup (see the note above). The jerk gap (0.07 vs 0.50) is one run each, across sessions, and Lighthouse jumps inflate jerk. Only say that second order reached the target with smooth commands and similar final spacing, until the like-for-like runs in §4 exist.
- Speed profile of the second-order run: it rises to 0.14 m/s at about 2 s, then falls steadily to about 0.01 m/s at the target. That's the "fast far, slow near" bell shape.
- Engineering lesson worth a paragraph: the first second-order version damped on measured Lighthouse velocity. Velocity noise times damping made commands flip direction every step (jerk 0.62–0.72) and the swarm shook. Damping on the commanded velocity plus a softer neighbour spring (`kappa_neighbour = 0.01`) fixed it (jerk 0.07). We checked this offline first with a delay and noise model.

### Problems seen on hardware (put in Discussion)
- Flying to start positions at stacked heights made drones lose Lighthouse tracking (position jumps of 1 m+ in 0.1–0.3 s, z reported below the floor). Fix: place by hand, `--placed`.
- Triangles put drones side by side along y, where the base stations are, so drones shadow each other and positions jump. Moving a base station to the x side would help.
- Battery below about 3.7 V caused radio link loss (one run at 3.26 V).

### Simulator (CrazySim, noise 0.03)

See `simulation-results.md`. What can go in the report:
- **Swarm, neighbour surprise:** spacing error 0.05 vs 0.23 m and safety push 0 % vs 37 % (batch 2); same pattern in batch 1 and on real drones. This is the most reliable result.
- **Two drones, first vs second order (same batch, same speed limit):** second order holds the distance in 2.7 s vs 2.9 s (adaptive) when too far and 1.8 s vs 6.8 s when too close, with about 45–55 % less jerk. n = 2–3.
- **Swarm, first vs second order (same start, same noise):** jerk 0.08 vs 0.95, a strong result. Arrival 11.8 s vs 25.1 s, but the speed limits still differ (0.15 vs 0.10 m/s), so say "probably faster", and do not quote 2×.
- **Fixed vs adaptive:** adaptive is about 2 s faster when too far in both batches. When too close, batch 1 favours it and batch 2 does not, so the result is mixed.
- **Fitts:** not confirmed. Swarm: a straight line fits better (R² 0.93 vs 0.89), which is what a speed limit produces. Pair: log is slightly better (0.98 vs 0.95), over a narrow range.
- **Convoy:** N = 0.05 freezes the followers in both batches (H2). H1 is **not supported**: no benefit in batch 2.
- **Formation:** converged in 10.2 s, one run (H3).
- Do not compare jerk or absolute arrival times between simulator and real drones.
- Say openly that many batch 2 runs failed from server stalls.

---

## 3. Report outline (4 pages)

| Section | Length | Content |
|---|---|---|
| Title + abstract | 0.2 p | Surprise minimisation for drone spacing and swarming. Key numbers: safety push 0 % vs 61 %, spacing error 0.10 vs 0.25 m (real), confirmed in simulation; second order about 12× lower command jerk in simulation |
| 1. Introduction | 0.4 p | Expectations → surprise → action. Why: decentralised spacing without a leader. Research questions (§3a below) |
| 2. Method | 1.0 p | Surprise \(S = \tfrac12(e/\sigma)^2\) as Gaussian surprisal. First-order laws (fixed / adaptive gain, gradient descent on \(F\)). Second-order law \(a = -k_p e - k_d \dot e\), \(k_d = 2\sqrt{k_p}\). Swarm version with one damper per spring. Fitts' law \(MT = a + b\log_2(D/W+1)\) and why a critically damped system follows it. Figure 1 |
| 3. Setup | 0.4 p | Crazyflie 2.x, Lighthouse V2, 10 Hz velocity setpoints, safety rules, scenarios (too far / too close, 3-drone start + target), metrics definitions |
| 4. Results | 1.4 p | Figures 2–4, Table 1. Two-drone comparison, swarm comparison, Fitts sweep |
| 5. Discussion | 0.4 p | Hardware issues (Lighthouse occlusion, transfer, battery); measured vs commanded damping; sim vs real agreement; limitations: few runs, surprise depends only on the current error, so the adaptive result shows a nonlinear gain helps (EXPERIMENTS.md §1) |
| 6. Conclusion + future work | 0.2 p | Restate the surprise-depends-on-error limitation. Predictive surprise, more drones |

### 3a. Research questions (use these as the backbone)
1. Does neighbour surprise keep a leaderless swarm spaced without relying on a safety rule? (goal_only vs surprise; real and sim agree, supported; more `goal_only` runs would help)
2. Does second-order control with matched damping move the drones faster and smoother than first order, without losing spacing? (first vs second order; sim supports it for two drones and for swarm smoothness, **swarm speed and all real data need like-for-like runs**)
3. Does the second-order controller follow Fitts' law (time grows with log of distance)? (Fitts sweep; **sim is inconclusive**, needs a sweep below the speed limit)
4. Does a surprise-adaptive gain correct spacing faster than a fixed gain? (fixed vs adaptive; **sim is mixed**, real needs same-session runs)

**Fitts range:** peak speed reaches the 0.15 m/s limit at about D = 0.8 m, so only
D = 0.4, 0.6 and 0.8 m are valid for the fit, and D = 1.2 m is speed-limited. That
is a narrow range: report R² with caution, or raise the limit for the sweep if it
is safe.

### 3b. Figures and table

| # | Figure | Source | Status |
|---|---|---|---|
| 1 | Method sketch: left, surprise \(S\) vs error for σ = 0.10; right, critically damped step response and its bell-shaped speed | Make once in matplotlib (5 lines), or reuse `data/simulation/euler_lagrange/distance_comparison.png` | to make |
| 2 | Two drones: distance vs time, too far and too close, colour = controller | `analysis/figures/second_order_pair_runs_real.png`, top row only (crop) | have, but missing `fixed` and `second_order_adaptive` |
| 3 | Swarm: (a) paths, (b) smallest pair distance vs time for goal_only / surprise. Add second_order after the like-for-like runs | `analysis/figures/target_swarm_runs_real.png`, `target_swarm_runs_sim.png`; later `second_order_swarm_runs_real.png` | have first order; second order needs re-flying |
| 4 | Fitts: (a) speed vs time for each target distance, (b) arrival time vs ID with fitted line and R² | bottom row of `second_order_swarm_runs_sim.png` | have sim data, inconclusive; redo after the sweep without the speed limit |
| T1 | Metrics per controller, mean ± std, real and sim side by side | `plot_second_order_*` tables and `python -m analysis.summarize_sim_runs` | have, n = 1–3 |

Keep figures two columns wide at most, 2 or 3 figures plus 1 table. That fits 4 pages.

---

## 4. Experiments still to run (priority order)

Fly each priority 1 group **in one session**, with the same start spots and speed
limit, batteries ≥ 3.9 V, drones **placed on the printed spots**.

⚠️ `target_swarm_experiment.py` has no `--placed` option yet, so add it first.
Otherwise first-order and second-order runs cannot start the same way.

| Priority | Runs | Command | Why |
|---|---|---|---|
| 1 | 3× | `python -m src.experiments.target_swarm_experiment --mode real --controller surprise --placed` (needs the option above) | First-order side of the fair comparison |
| 1 | 3× | `python -m src.experiments.second_order_swarm_experiment --mode real --controller second_order --placed --set max_horizontal_speed=0.10` | Same start and speed limit, so research question 2 can be answered |
| 1 | 2× more | `python -m src.experiments.target_swarm_experiment --mode real --controller goal_only --placed` | Research question 1 needs n ≥ 3 |
| 1 | 3× | `python -m src.experiments.second_order_swarm_experiment --mode real --controller goal_only_second_order --placed --set max_horizontal_speed=0.10` | Second-order baseline |
| 2 | 3× each at D = 0.4, 0.6, 0.8 | `python -m src.experiments.second_order_swarm_experiment --mode real --controller second_order --placed --target-distance 0.4` (etc.) | **Fitts' law has no data**; research question 3 |
| 3 | 3× each scenario | `python -m src.experiments.surprisal_goal_navigation_experiment --mode real --controller fixed --scenario too_far` (and `too_close`), same session as adaptive | Research question 4 on real drones |
| 3 | 3× each scenario | `python -m src.experiments.second_order_pair_experiment --mode real --controller second_order_adaptive --scenario too_far --placed` (and `too_close`) | Surprise-adaptive second order never flown |

**Simulator, on the server.** Fix the stalls first (`simulation-results.md` §8): move the repo off `/mnt/d`, and install matplotlib. Then:

| Priority | Command | Why |
|---|---|---|
| 1 | `N_DRONES=3 ./scripts/run_experiment.sh second_order_swarm --mode sim --controller second_order --set max_horizontal_speed=0.10 --noise 0.03 --seed 101` (seeds 101–103) | Equal speed limit settles swarm speed |
| 1 | `ONLY=target_swarm ./scripts/run_all_sim.sh 2>&1 \| tee sim_target.log` | Only 1 `surprise` run in batch 2 |
| 2 | `N_DRONES=3 ./scripts/run_experiment.sh second_order_swarm --mode sim --controller second_order --target-distance 0.8 --set max_horizontal_speed=0.40 --noise 0.03 --seed 101` (D = 0.4–2.0, seeds 101–103) | Fitts without the speed limit |
| 3 | `ONLY=goal_navigation ./scripts/run_all_sim.sh 2>&1 \| tee sim_pair1.log` | Settles adaptive when too close |

The convoy can be mentioned in one short paragraph: the N = 0.05 freezing result
is solid, and H1 is not supported. There is no real-drone convoy data.

---

## 5. Analysis still to do

- [x] Regenerate all real figures (done: `plot_second_order_pair_runs`, `plot_second_order_swarm_runs`, `plot_goal_navigation_runs`, `plot_target_swarm_runs`).
- [x] Sim figures and `summarize_sim_runs` for the current sim data (two-drone, swarm, convoy). `plot_convoy_runs` and `plot_formation_runs` have no real data, which is fine.
- [ ] Add `--placed` to `target_swarm_experiment.py`.
- [ ] Make the two two-drone plots count runs the same way (`as_placed` runs).
- [x] `analysis/summarize_second_order_sim.py` for batch 2 (first vs second order, Fitts log and linear fits).
- [ ] Use mean spacing over the last 5 s for `final_spacing_error`; the last-sample value misleads when drones bounce.
- [ ] Re-run all `--mode sim` scripts after the next batch.
- [ ] **Use a better "arrival" metric for the two-drone Fitts plot.** `settling_time` ("never leaves the band again") is ruined by jitter, and "first time in the band" is too noisy: one too-close run passes through the band at 0.2 s during the start wobble. `hold_time` in `analysis/summarize_sim_runs.py` (stays within ±8 cm for 2 s) already does this; move it into `plot_second_order_pair_runs`. The swarm's `arrival_time` (centre within 10 cm) works as it is.
- [ ] Re-run both `plot_second_order_*` scripts after the new flights. The Fitts panels fill in automatically.
- [ ] Optional: `python -m analysis.plot_surprisal_distance data/raw/<best second-order pair run>.csv --target 0.6` for a single clean example run.
- [ ] Figure 1 (method sketch); not produced by any script yet.
- [ ] Statistics: with 3 runs per condition, report mean ± std only; do not claim significance.

---

## 6. Do we need more experiments?

**Yes.** The status of each research question:

| Question | Status | Minimum still needed |
|---|---|---|
| 1. Neighbour surprise keeps the swarm spaced | **Supported**: real and sim agree | `goal_only` ×2 more (real) |
| 2. Second order vs first order | **Sim supports it**, except swarm speed; real comparison is not fair | Sim at equal speed limit; like-for-like real flights (§4, priority 1) |
| 3. Fitts' law | **Inconclusive** | Sim sweep below the speed limit; real sweep at D = 0.4, 0.6, 0.8 |
| 4. Adaptive vs fixed gain | **Mixed** | Repeat sim `goal_navigation`, or report as mixed |

If time is short, do the simulator runs first, because they carry no crash risk,
then priority 1 real flights. Research question 1 and the simulator's second-order
results can carry the report if the real second-order flights do not work out.
