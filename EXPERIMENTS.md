# Drone-Surprisal — Experiment Brief

> **How to use this file.** This is the single source of truth for *what* we are
> testing and *why*. Code in the repo should implement what is written here; when
> an experiment changes, update this file first, then the code. Hand this file to
> any collaborator (human or Claude Code) before they touch the experiment
> scripts.
>
> **Status.** Sections 1–6 were reconciled with the actual code and confirmed by
> the team. Items still marked `[FILL IN]` are decisions not made yet (mostly
> numeric thresholds and sweep values). Items marked *(planned)* describe
> features that are specified here but not implemented yet.

---

## 1. Project overview

- **Concept:** surprise minimisation in a **leader–follower convoy** of three
  Crazyflies. Each follower *predicts* that it should move at the same speed as
  the drone ahead of it. When that prediction is violated (the drone ahead
  suddenly changes speed), the follower is "surprised" and slows its own
  movement down. This is a simplified, active-inference-inspired rule: there is
  no generative model or belief update — just a prediction error that gates the
  action.
- **Plain-language goal:** show that followers that slow down when surprised
  keep safer spacing when the leader stops abruptly than followers using plain
  P-control — and find the range of the surprise sensitivity *N* where this
  helps without freezing the convoy.
- **Hypotheses:**
  - **H1:** after the leader stops, the surprise-gated followers (E1) have
    smaller spacing overshoot and no minimum-spacing violations compared with
    the baseline (E2).
  - **H2 (sensitivity):** the benefit depends on *N*; there is a range of *N*
    where gating helps **without** freezing the convoy.
  - **H3:** the formation-potential controller (E3) converges to the target
    spacing.
- **Proposed vs baseline:**
  - Proposed — **E1**, surprise-gated convoy
    (`src/experiments/surprise_minimization_experiment.py`).
  - Baseline — **E2**, the same convoy with plain P-control
    (`src/experiments/convoy_baseline_experiment.py`).
  - **The only difference** is the `exp(−k·U)` speed factor. Both use the same
    controller function (`use_surprise=True/False`), the same config file and
    the same control loop.
- **Separate experiment:** **E3**, formation-distance gradient descent
  (`src/experiments/formation_distance_experiment.py`). It is not a baseline
  for E1.

---

## 2. Key definitions (convoy, E1/E2)

These match `src/controllers/convoy_controller.py`. If the two ever disagree,
this file wins and the code is a bug.

All quantities are along the **x axis** (the convoy direction). Follower *i*
follows drone *i−1* (CF2 follows CF1, CF3 follows CF2).

- **Observation `o`:** the drone's own estimated state from the onboard
  estimator — `stateEstimate.x` and `stateEstimate.vx` — for itself and for the
  drone ahead. In sim this comes from CrazySim; on hardware from Lighthouse.
  No Multiranger or Flow deck is used.
- **Prediction `ô`:** follower *i* expects to move like the drone ahead:
  `v̂_i = v_{i−1}`.
- **Prediction error:** `S_i = v_{i−1} − v_i` (m/s).
- **Surprise:** `U_i = ½ · (S_i / N)²` (dimensionless).
  *N* (`noise_scale`) is the speed difference that counts as "normal";
  default **N = 0.05 m/s**.
- **Speed factor:** `f_i = exp(−k · U_i)`, with `k` = `surprise_gain` = **2.0**.
  `f = 1` → no surprise, move normally; `f → 0` → highly surprised, hold still.
- **Action `a`:** a world-frame x-velocity setpoint
  (`commander.send_velocity_world_setpoint(vx, 0, 0, 0)`).
- **Follower policy (each control step):**
  1. P-control towards the spot `follow_distance` behind the drone ahead:
     `base = clamp(follow_gain · ((x_{i−1} − d) − x_i), ±max_follow_speed)`.
  2. E1 only: `command = base · f_i`. E2: `command = base`.
  3. **Safety repulsion (E1 and E2):** if the gap `x_{i−1} − x_i` is smaller
     than `safe_distance`, `command = −max_follow_speed` (back away). This runs
     last, so surprise gating can never cancel it.
- **Leader policy:** fly at `leader_speed` towards a **virtual wall** at
  `wall_x`; once within `wall_stop_distance` of it, stop (velocity 0) for the
  rest of the run. The wall is only a number in the config — it is the trigger
  that makes the leader stop abruptly. There is no physical obstacle and no
  range sensing.

**Known issue (motivates H2):** with N = 0.05, a speed mismatch of only
0.1 m/s gives U = 2 and f ≈ 0.02, so the followers almost freeze. A legacy run
logged U = 100 → f ≈ 1e-88. Before any E1-vs-E2 comparison, check the
speed-factor diagnostic (§8) and sweep *N* into a meaningful range.

### E3 definitions (formation distance)

Matches `src/controllers/formation_controller.py`.

- Gap errors: `S12 = (x2 − x1) − d`, `S23 = (x3 − x2) − d` (signed).
- Formation potential: `U = ½ (S12² + S23²)` (m²; 0 = perfect).
- Gradient-descent step per drone:
  `Δx1 = g·S12`, `Δx2 = g·(S23 − S12)`, `Δx3 = −g·S23`, each clamped to
  `±max_step`; sent as a `go_to` target over one control period.

Note: E3's *U* is a formation potential, not the convoy's velocity surprise —
they share a letter but are different quantities.

---

## 3. Platform & stack (do not change without noting it here)

- **Simulator:** CrazySim — software-in-the-loop (SITL) Crazyflie firmware
  coupled to Gazebo. Launched with `sitl_multiagent_square.sh -n 3`.
- **Control interface:** CFLib (`crazyflie-lib-python`). Sim URIs are
  `udp://127.0.0.1:1985N` for drone N (19850 = CF1). Real URIs are radio
  addresses `[FILL IN]` in `config/default.yaml`.
- **Sensors / positioning:** only the onboard `stateEstimate` (x, y, z, vx,
  vy, vz). **No decks are read.**
  - Sim: state comes from CrazySim.
  - Real: state comes from a **Lighthouse** positioning system, which gives
    absolute x / vx directly. The controllers therefore need **no changes**
    for hardware; no Multiranger or Flow deck is needed.
- **Number of drones:** 3 for E1, E2, E3. Sanity tests use 1 or 3.
- **Rule:** controllers (`src/controllers/`) are pure maths and must stay
  identical for sim and real — no sim-only logic in the control path.

---

## 4. Shared experimental setup

Values live in `config/default.yaml` and `config/experiments/*.yaml`.

| Item | Convoy (E1/E2) | Formation (E3) | Notes |
|---|---|---|---|
| Altitude | 0.5 m | 0.5 m | `default.yaml: height` |
| Takeoff / land time | 2.0 s / 2.0 s | same | `default.yaml` |
| Start line (x of CF1, CF2, CF3) | 0.0, −0.5, −1.0 | 0.0, 0.3, 1.1 | E3 starts deliberately off-target |
| Control rate | 10 Hz (0.1 s) | ≈3.3 Hz (0.3 s) | `control_period` |
| Telemetry rate | 10 Hz | 10 Hz | `default.yaml: log_period_ms: 100` |
| Run duration | 25 s | 15 s | `duration` |
| Arena | CrazySim multi-agent square world; virtual wall at x = 2.0 | same world | |
| Repetitions | see below | see below | |
| Randomness | none by default — the sim is deterministic | | |

**Repetitions and seeds (planned):** with observation noise **off**, the sim is
deterministic, so **1 run per condition**. With noise **on**, run several
seeds per condition `[FILL IN how many, e.g. 10]`. Noise level and seed are set
in config and recorded in the metadata sidecar.

**Variable vocabulary (use these terms everywhere):**

- **Independent variables (IV):** what we deliberately change between conditions.
- **Controlled variables:** what we hold fixed so comparisons are fair.
- **Dependent variables (DV):** what we measure to judge the outcome.

---

## 5. Experiments

### E1 — Surprise-gated convoy (PROPOSED) · `src/experiments/surprise_minimization_experiment.py`

- **Objective:** test whether surprise gating makes followers keep safer
  spacing when the leader stops abruptly (H1), and for which *N* (H2).
- **Drones & positions:** CF1 (leader) at x = 0.0, CF2 at −0.5, CF3 at −1.0,
  all at 0.5 m altitude, in a line along x.
- **Environment:** empty sim world; virtual wall at x = 2.0. The leader stops
  0.5 m before it (x ≈ 1.5).
- **Procedure:**
  1. Connect, take off to 0.5 m, fly to the start line.
  2. For 25 s at 10 Hz: leader flies at 0.15 m/s and stops at the virtual
     wall; followers use the surprise-gated P-control of §2.
  3. Set all velocities to zero, land, save the log.
- **Independent variables:** noise scale *N* `[FILL IN sweep values]`;
  leader speed `[FILL IN]`; wall position / stop distance `[FILL IN]`;
  observation noise level (optional) `[FILL IN]`.
- **Controlled variables:** everything else in `convoy.yaml` (gains, follow
  distance, safe distance, start line, duration, rates).
- **Dependent variables:** see §8 per-run metrics.
- **Extra logged columns:** gaps, `S`, `U`, speed factor, repel flag and
  command per follower (see §7).
- **Success criteria:** `[FILL IN — e.g. lower overshoot than E2 and zero
  collisions, for at least one N where the convoy does not freeze]`.
- **Real-drone note:** none beyond the general rules (Lighthouse, conservative
  speed, kill switch).

### E2 — Convoy baseline · `src/experiments/convoy_baseline_experiment.py`

- **Objective:** the baseline E1 is compared against.
- **Method:** identical to E1 — same config, same loop, same controller
  function — but with `use_surprise=False`, so followers use plain P-control
  (plus the same safety repulsion). *S*, *U* and the speed factor are still
  computed and logged so both experiments have the same columns, but they do
  not affect control.
- **Drones / environment / procedure / IV / controlled / DV:** identical to E1.
  Every sweep must be run for both E1 and E2 with the same values.
- **Change from legacy:** the legacy `wall_follow_experiment.py` also locked a
  follower at zero speed once it had settled after the leader stopped. That
  lock was removed so the **only** difference from E1 is the gating.
- **Success criteria:** n/a (reference).

### E3 — Formation distance · `src/experiments/formation_distance_experiment.py`

- **Objective:** test whether gradient descent on the formation potential
  brings the neighbour gaps to the target spacing (H3).
- **Drones & formation:** 3 drones in a line along x, target gap 0.5 m.
  Start line 0.0 / 0.3 / 1.1 (gaps 0.3 m and 0.8 m).
- **Procedure:**
  1. Connect, take off, fly to the (wrong) start line.
  2. For 15 s, every 0.3 s: compute one gradient step (§2) and send each
     drone to its new x with `go_to`.
  3. Land, save the log.
- **Independent variables:** `[FILL IN — e.g. start-line perturbation, gain]`
- **Controlled variables:** desired distance, max step, duration, rates.
- **Dependent variables:** final gap error, time for both gaps to stay within
  `[FILL IN]` m of 0.5, final *U*.
- **Extra logged columns:** gaps, `S12`, `S23`, `U`, step and target per drone.
- **Success criteria:** `[FILL IN — e.g. both gaps within 0.05 m of 0.5 m by
  the end of the run]`.

---

## 6. Sanity / test scripts (keep these working — do not break)

Not experiments; they verify the rig before a session. Run from the repo root.

- `python -m src.tests.test_connection` — connects, streams positions for 5 s,
  prints PASS/FAIL. **Never starts the motors.** `--drones 1` for CF1 only.
- `python -m src.tests.takeoff_test` — CF1 takes off, hovers 3 s, lands.
- `python -m src.tests.three_takeoff_test` — 3 drones take off, hover 5 s, land.
- `python -m src.tests.hover_telemetry_test` — CF1 hovers and prints its
  position for 6 s; PASS if within 0.1 m of target height.

All flights go through `fly_safely()` (`src/utils/flight.py`), which always
lands — on normal finish, error or Ctrl-C — and calls
`send_notify_setpoint_stop()` before landing.

---

## 7. Data logging standard *(planned — not implemented yet)*

The experiments currently save one simple wide CSV per run
(`data/raw/<experiment>_<timestamp>.csv`, one row per control step). The
standard below will replace it so one analysis script can read every
experiment.

**File naming:** `data/raw/<experiment>_<YYYYMMDD-HHMMSS>_run<NN>.csv`
Each CSV has a sidecar `….json` with the run's metadata (see below).

**Metadata sidecar (per run):**
```json
{
  "experiment": "surprise_minimization",
  "timestamp": "2026-10-06T14:00:00",
  "mode": "sim",
  "n_drones": 3,
  "git_commit": "<hash>",
  "params": { "noise_scale": 0.05, "leader_speed": 0.15 },
  "obs_noise_std": 0.0,
  "seed": 42,
  "log_rate_hz": 10,
  "notes": ""
}
```

**Core column schema (every experiment, one row per drone per timestep):**

| Column | Unit | Meaning |
|---|---|---|
| `t` | s | time since the control loop started |
| `drone_id` | — | 0 = CF1, 1 = CF2, 2 = CF3 |
| `x, y, z` | m | estimated position |
| `vx, vy, vz` | m/s | estimated velocity |
| `cmd_type` | — | setpoint type sent (`vel` / `pos`) |
| `cmd_x, cmd_y, cmd_z` | m or m/s | commanded setpoint |
| `battery_v` | V | battery voltage (real); sim may be constant |

Experiments **add** their own columns (listed in §5). Never rename core columns.

**Rules:**
- One run = one CSV. Never append two runs into one file.
- Log at a fixed rate; record that rate in the metadata.
- Write the metadata sidecar even for throwaway runs.
- Raw data in `data/raw/` is never hand-edited. Pre-migration CSVs are kept
  unchanged in `data/raw/legacy/` and are not converted.

---

## 8. Analysis plan *(planned)*

- **Per-run metrics (convoy, E1/E2):**
  - minimum inter-drone spacing;
  - spacing overshoot vs the 0.5 m target;
  - settling time after the leader stops;
  - collision flag — spacing below `[FILL IN threshold]` m at any time;
  - final spacing error;
  - total control effort (Σ |command| · Δt);
  - **speed-factor diagnostic:** fraction of steps where `f < 0.1`, per
    follower — shows whether gating is meaningful or just stopping the drones;
  - fraction of steps where the safety repulsion was active.
- **Per-run metrics (E3):** final gap error, convergence time, final *U*.
- **Aggregation:** per condition → mean ± std across seeds (noise on), or the
  single value (noise off).
- **Core comparison:** E1 vs E2 on the same metrics, same conditions, across
  the *N* sweep (H1, H2).
- **Standard plots:**
  - gaps over time, E1 vs E2;
  - surprise and speed factor over time;
  - metric vs *N* (and vs leader speed), E1 and E2 on the same axes.
- **Statistics (optional):** `[FILL IN — e.g. paired comparison across seeds;
  report effect size]`.
- Analysis reads only `data/raw/` + sidecars, writes figures to
  `analysis/figures/`, and never flies a drone.

---

## 9. Deliverables & definition of done

- [x] All scripts migrated into `src/` using shared `src/utils/` helpers.
- [ ] Sanity tests and E1/E2/E3 confirmed flying in CrazySim.
- [ ] Every experiment produces CSVs in the core schema + metadata sidecar.
- [ ] Config-driven sweeps (N, leader speed, wall position), identical for E1
      and E2, with optional observation noise and seeds.
- [ ] `analysis/` reproduces the standard plots from `data/raw/` with one command.
- [ ] E1 vs E2 comparison plotted and its success criterion evaluated.
- [ ] Each experiment runs in sim **and** has a documented path to real
      hardware (Lighthouse; see `README.md`).

---

## 10. Open decisions

1. Collision threshold for the metrics (§8) — and is `safe_distance = 0.3 m`
   (the repulsion trigger in `convoy.yaml`) the right value?
2. Sweep values for *N*, leader speed and wall position / stop distance (§5).
3. Observation-noise levels and number of seeds per condition (§4).
4. Success criteria for E1 and E3 (§5).
5. Real-drone radio URIs (`config/default.yaml`).
6. Exact package versions (`requirements.txt`, from `pip freeze` on the sim
   machine).
