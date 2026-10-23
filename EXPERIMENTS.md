# Drone-Surprisal — Experiment Brief

> **How to use this file.** This is the source of truth for *what* we test and
> *why*. When an experiment changes, update this file first, then the code.
> How to *run* things is in [README.md](README.md). Every number below lives in
> `config/default.yaml` or `config/experiments/<name>.yaml` — the YAML wins if
> the two ever disagree.

---

## 1. Idea

Each drone carries **expectations** (a desired distance to a neighbour, a speed
it expects the drone ahead to have, a target it expects to be at). When an
observation differs from the expectation, the drone is **surprised**. We use a
Gaussian surprise

    S = 1/2 * (error / sigma)^2

which is, up to a constant, −log p(error) for a normal distribution with spread
σ — the information-theoretic *surprisal*. σ says how big an error counts as
"normal". The experiments test different ways a drone can **act on surprise**:

| Experiment | Expectation | What surprise does |
|---|---|---|
| E1 convoy | "the drone ahead moves at my speed" | **slows** the follower down |
| Two-drone distance | "my partner is 0.6 m away" | makes the correction **stiffer** |
| Target swarm | "I am at the target" + "neighbours are 0.6 m away" | drones **move to minimise** their total surprise |
| E3 formation | "neighbours are 0.5 m away" | gradient descent on a formation potential |

**Honest caveat.** Where surprise depends *only* on the current error (two-drone
`adaptive`, target swarm), it is mathematically a nonlinear function of that
error; the comparison with the baseline shows the effect of that nonlinearity.
Showing that *surprise as such* matters needs surprise that depends on more than
the error — e.g. a prediction error (`d_measured − d_predicted`) or a learned σ.
That is the planned next step (§8).

---

## 2. Platform

- **Simulator:** CrazySim (Crazyflie SITL firmware + Gazebo), on the Linux server.
- **Hardware:** Crazyflie 2.x, Lighthouse deck, 2 Lighthouse V2 base stations
  (at y ≈ −3.2 m and y ≈ +3.0 m), one Crazyradio.
- **Interface:** cflib. One switch, `--mode sim|real`; controllers are identical.
- **Sensing:** the onboard state estimate only (`stateEstimate.x/y/z/vx/vy/vz`),
  from CrazySim in sim and from Lighthouse on hardware. No Multiranger/Flow.
- **Flight:** takeoff 0.4 m (high-level commander), experiments command world-frame
  velocities at 10 Hz, landing waits for touchdown. Safety: see README.

**Simulator realism.** CrazySim is deterministic. Sim runs use `--noise 0.03`
(Gaussian noise, std 3 cm / 3 cm/s, on what the controller sees) with a recorded
seed per run, so repeated runs differ the way real ones do. Safety and the logged
"true" positions are never noisy.

---

## 3. Two-drone surprisal distance — `surprisal_goal_navigation`

**Question:** does a gain that grows with surprise restore the spacing faster
and more accurately than a fixed gain, without getting closer?

**Setup.** 2 drones. Desired distance **0.60 m**. Two start scenarios:
`too_far` (0.80 m apart → must move together) and `too_close` (0.45 m apart →
must move apart). Sim: drones fly to these start positions after takeoff. During
the run the pair's midpoint also moves 0.5 m in +x (goal).

**Per step:**

    d   = distance between the drones (horizontal)
    e   = d − 0.60                                   (>0 too far, <0 too close)
    S   = 1/2 (e / 0.10)^2
    kp  = 0.25                                       fixed     (baseline)
    kp  = min(0.25 (1 + 1.5 S), 0.60)                adaptive  (proposed)
    correction = kp e + 0.25 * (rate of change of d)   (damping)
    each drone moves ½·correction towards (e>0) / away from (e<0) the other,
    plus the shared goal velocity; below 0.40 m a safety push overrides; below
    0.30 m the run aborts.

**Conditions:** {fixed, adaptive} × {too_far, too_close}, 3+ runs each.

**Metrics** (per run, in the JSON and summary CSV):
`settling_time` (|e| stays ≤ `distance_tolerance` from then on), `rms_error`,
`overshoot` (past 0.6 m), `min_distance`, `control_effort`, `protective_fraction`.

**Hypothesis H-D:** adaptive has lower settling time and RMS error than fixed in
both scenarios, with equal or larger `min_distance`.

**Real-drone results so far (19 Oct):** fixed, too_far 0.87 → 0.60 m, and fixed,
too_close 0.43 → 0.60 m, both reached the target within ~11 s.
Note: with real jitter (±4 cm) `settling_time` is inflated by brief excursions
from the ±5 cm band — consider `distance_tolerance: 0.08`.

---

## 4. Three drones to a target — `target_swarm` (new)

**Question:** can drones reach a shared target *and* keep safe, even spacing
with no leader, just by each minimising its own surprise?

**Setup.** 3 drones start at (−0.8, −0.6), (−0.8, 0), (−0.8, 0.6); target (0.6, 0).

**Surprise of drone i:**

    S_goal_i = 1/2 (|p_i − target| / 0.30)^2
    S_ij     = 1/2 ((d_ij − 0.60) / 0.10)^2        for each other drone j
    F_i      = S_goal_i + Σ_j S_ij                 (surprise)
    F_i      = S_goal_i                            (goal_only, baseline)
    v_i      = −0.02 · ∇F_i,   limited to 0.10 m/s

Below 0.35 m a safety push (not part of the surprise) separates any pair; below
0.20 m the run aborts.

**Expected:** `surprise` → the drones settle in a triangle around the target with
sides ≈ 0.58 m (the target pull compresses it slightly); `goal_only` → they crowd
together and only the safety push keeps them apart.

**Metrics:** `arrival_time` (swarm centre within 0.10 m of the target),
`final_centre_error`, `min_pair_distance`, `final_spacing_error`
(mean |d_ij − 0.6|), `final_spacing_spread` (0 = even triangle),
`final_total_surprise`, `protective_fraction`, `control_effort`.

**Hypothesis H-T:** `surprise` gives a larger `min_pair_distance`, far smaller
`final_spacing_error` and `protective_fraction`, at a similar `arrival_time`.

**Offline check (fake drones, no noise):** surprise → triangle 0.58 m, 0 % safety
push; goal_only → 0.33 m spacing, safety push active 83 % of the time.

---

## 5. Convoy — E1 `surprise_minimization` vs E2 `convoy_baseline`

**Setup.** 3 drones in a line along x (sim: x = 0, −1, −2; real: `convoy_real.yaml`).
CF1 (leader) flies +x at 0.15 m/s and stops 0.5 m before a virtual wall
(sim: wall at x = 5.0). CF2 follows CF1, CF3 follows CF2, target gap
`follow_distance` (sim 1.0 m).

**Follower i per step:**

    base     = clamp(0.8 · ((x_ahead − gap) − x_i), ±0.15)        P-control
    S_i      = v_ahead − v_i                                       prediction error
    U_i      = 1/2 (S_i / N)^2                                     surprise, N = noise_scale
    command  = base · exp(−k U_i)      E1 (k = surprise_gain)
    command  = base                    E2
    below safe_distance: back away at full speed (both)

E1 and E2 share all code; the only difference is the `exp(−k U)` factor.

**Metrics:** `min_gap`, `stop_time`, `overshoot_after_stop` (how far a gap went
below the target after the leader stopped), `settling_after_stop`,
`final_gap_error`, `low_factor_fraction` (share of steps with factor < 0.1 —
"is gating just freezing the drones?"), `repel_fraction`.

**Hypotheses:**
- **H1:** after the leader stops, E1 has smaller overshoot and no gap below
  `safe_distance`, compared with E2.
- **H2:** this depends on N; there is a range of N where E1 helps without
  freezing (low `low_factor_fraction`). The batch sweeps N ∈ {0.05, 0.1, 0.2, 0.4}.

---

## 6. Formation — E3 `formation_distance`

3 drones on a line start at x = 0, 0.3, 1.1 (gaps 0.3 and 0.8 m) and correct
their gaps by gradient descent on U = ½(S12² + S23²), S = gap − 0.5 m, using
`go_to` steps every 0.3 s.

**Metrics:** `convergence_time`, `final_gap_error`, `final_U`, `min_gap`.
**H3:** the gaps converge to 0.5 m and U → 0.

---

## 7. Data and analysis standard

- One run = `data/raw/<experiment>_<controller>_<mode>_<time>.csv` (10 Hz rows,
  **true** positions plus what the controller computed) + a `.json` sidecar
  (mode, scenario, noise, seed, aborted, git commit, full config, metrics).
- Aborted runs are kept but excluded from every analysis.
- `analysis/plot_*_runs.py --mode sim|real`: figure in `analysis/figures/` and a
  mean ± std table per controller (and scenario / N).
- Comparisons are always between controllers under **identical** conditions
  (same config, scenario, noise level; seeds paired by repeat number in the batch).

---

## 8. Status and next steps

- [x] All experiments run through `fly_safely()` with pre-/in-flight safety.
- [x] Real hardware: sanity tests, two-drone experiment (fixed) confirmed.
- [x] Sim-ready: start positions, noise + seeds, CSV + JSON per run, batch script,
      analysis for every experiment.
- [ ] Sim batch on the server (`scripts/run_all_sim.sh`) and first sim results.
- [ ] Real runs: two-drone `adaptive`; target swarm; convoy with `convoy_real.yaml`.
- [ ] **Predictive surprise** (surprise from prediction error, not error size) as a
      third controller, to show surprise itself — not just a nonlinear gain —
      changes behaviour.
- [ ] Statistics: paired comparison across seeds, effect sizes.

`src/experiments/surprisal_hover_distance_experiment.py` is **not maintained**: its
"responder" logic can swap roles back and forth. Superseded by the two-drone
goal-navigation experiment.
