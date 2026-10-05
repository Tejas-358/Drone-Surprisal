# Drone-Surprisal — Experiment Brief

> **How to use this file.** This is the single source of truth for *what* we are
> testing and *why*. Code in the repo should implement what is written here; when
> an experiment changes, update this file first, then the code. Hand this file to
> any collaborator (human or Claude Code) before they touch the experiment
> scripts.
>
> **⚠️ Assumptions to confirm.** This brief was drafted from the repository
> structure, not from a verified project description. Everything marked
> `[CONFIRM]` or `[FILL IN]` must be checked by the team before relying on it.
> The core assumption is that this project is about **active inference / the free
> energy principle**, i.e. the drone acts to *minimise surprise* (the mismatch
> between what it expects to sense and what it actually senses). If that is wrong,
> correct Section 1 and 2 first — everything else follows from them.

---

## 1. Project overview

- **Concept / framework:** Surprise minimisation (active inference / free energy
  principle). The agent selects actions that reduce the discrepancy between its
  predicted sensory input and its actual sensory input. `[CONFIRM]`
- **Plain-language goal:** `[FILL IN — one or two sentences on what the project
  is trying to demonstrate, e.g. "that a surprise-minimising controller keeps a
  drone at a target distance from a wall more robustly than a fixed controller,
  especially under sensor noise."]`
- **Primary hypothesis (H1):** `[FILL IN — a falsifiable statement, e.g.
  "The surprise-minimising controller maintains target distance with lower RMS
  error than the baseline PID controller when sensor noise is increased."]`
- **Secondary hypotheses (optional):** `[FILL IN]`
- **Baseline vs proposed method:**
  - Baseline: `distance_controller.py` `[CONFIRM — is this the fixed/PID baseline?]`
  - Proposed: `surprise_minimization_experiment.py` `[CONFIRM]`
  - What differs between them: `[FILL IN]`

---

## 2. Key definitions (the heart of the project)

These must match the implementation exactly. Fill them from the code.

- **Observation `o`:** what the drone actually senses each step.
  `[CONFIRM — likely Multiranger deck ranges (front/back/left/right/up) and/or
  Flow-deck position/velocity.]`
- **Expectation / prediction `ô`:** what the drone's internal (generative) model
  expects to sense. `[FILL IN — e.g. expected range = target distance to wall.]`
- **Surprise / surprisal:** the quantity being minimised.
  `[FILL IN — the exact operational definition used in code, e.g. squared
  prediction error  surprise = (ô − o)²  summed over active sensors, or a
  variational free-energy proxy. State units and which sensors contribute.]`
- **Action `a`:** what the controller outputs. `[CONFIRM — velocity setpoints /
  position setpoints / high-level commander.]`
- **Policy / update rule:** how actions are chosen from surprise.
  `[FILL IN — e.g. gradient descent on surprise, or sampling candidate actions
  and picking the one with lowest predicted surprise.]`

> Keep the maths here consistent with whatever is in `surprise_minimization.py`.
> If the two ever disagree, this file wins and the code is a bug.

---

## 3. Platform & stack (do not change without noting it here)

- **Simulator:** CrazySim — software-in-the-loop (SITL) for the Crazyflie nano
  quadrotor. Runs the real Crazyflie firmware on the desktop and couples it to
  Gazebo for physics and sensors.
- **Control interface:** CFLib (`crazyflie-lib-python`). The **same scripts must
  run in simulation and on real Crazyflies** — see the sim-to-real rules in
  `README.md`. Do not add sim-only shortcuts into the control logic.
- **Assumed decks / sensors:** Multiranger (5× ToF range) + Flow deck v2
  (optical-flow position/velocity). `[CONFIRM — which decks are actually used?]`
- **Number of agents:** single-drone and multi-drone experiments both exist
  (`takeoff_test.py` vs `three_takeoff_test.py`, `formation_test.py`).

---

## 4. Shared experimental setup

Defaults that apply to every experiment unless overridden in its section.

| Item | Value | Notes |
|---|---|---|
| Default altitude | `[FILL IN]` m | e.g. 0.5 m |
| Arena size | `[FILL IN]` | sim world bounds |
| Control rate | `[FILL IN]` Hz | how often a setpoint is sent |
| Logging rate | `[FILL IN]` Hz | should match or divide control rate |
| Run duration | `[FILL IN]` s | per single run |
| Repetitions per condition | `[FILL IN]` | e.g. 10 — needed for statistics |
| Random seed handling | `[FILL IN]` | fix seeds for reproducibility |

**Variable vocabulary (use these terms everywhere):**

- **Independent variables (IV):** what we deliberately change between conditions.
- **Controlled variables:** what we hold fixed so comparisons are fair.
- **Dependent variables (DV):** what we measure to judge the outcome.

---

## 5. Experiments (mapped to the repo files)

Each experiment uses the same sub-structure so results stay comparable and
analysis can be automated.

### E1 — Surprise minimisation (CORE) · `surprise_minimization_experiment.py`

- **Objective:** `[FILL IN]`
- **Hypothesis tested:** H1 `[CONFIRM]`
- **Drones & positions:** `[FILL IN]`
- **Environment:** `[FILL IN — wall/obstacle layout the drone senses]`
- **Procedure (step by step):**
  1. Connect, take off to default altitude.
  2. `[FILL IN the behaviour, e.g. hold target distance from the wall while the
     target distance / noise is varied.]`
  3. Land, save log.
- **Independent variables:** `[FILL IN — e.g. target distance ∈ {0.3, 0.5, 0.8} m;
  sensor-noise level ∈ {none, low, high}]`
- **Controlled variables:** `[FILL IN]`
- **Dependent variables (measures):** `[FILL IN — e.g. RMS distance error,
  mean surprise, control effort, settling time]`
- **Logged columns:** see Section 7 core schema **plus**
  `expected_range, measured_range, surprise, action_cmd` `[CONFIRM/EXTEND]`
- **Success criteria:** `[FILL IN]`
- **Real-drone note:** `[FILL IN anything that must change for hardware, ideally
  nothing but conservative limits]`

### E2 — Distance control baseline · `distance_controller.py` / `distance_experiment.csv`

- **Objective:** provide the baseline that E1 is compared against. `[CONFIRM]`
- **Method & how it differs from E1:** `[FILL IN]`
- **Drones / environment / procedure:** `[FILL IN]`
- **IV / controlled / DV:** `[FILL IN — keep identical to E1 so the comparison
  is fair]`
- **Logged columns:** core schema + `target_distance, measured_distance,
  action_cmd` `[CONFIRM/EXTEND]`
- **Success criteria:** `[FILL IN]`

### E3 — Wall following · `wall_follow_experiment.py` / `wall_follow_experiment.csv`

- **Objective:** `[FILL IN]`
- **Setup (wall position, side followed):** `[FILL IN]`
- **Procedure:** `[FILL IN]`
- **IV / controlled / DV:** `[FILL IN]`
- **Logged columns:** core schema + `wall_side, measured_range, lateral_error`
  `[CONFIRM/EXTEND]`
- **Success criteria:** `[FILL IN]`

### E4 — Formation · `formation_test.py`

- **Objective:** `[FILL IN]`
- **Number of drones & formation shape:** `[FILL IN — e.g. 3 drones, triangle,
  0.8 m spacing]`
- **Procedure:** `[FILL IN]`
- **IV / controlled / DV:** `[FILL IN — e.g. inter-drone distance error]`
- **Logged columns:** core schema (one row per drone per step) + `formation_id,
  neighbour_id, desired_spacing, measured_spacing` `[CONFIRM/EXTEND]`
- **Success criteria:** `[FILL IN]`

---

## 6. Sanity / test scripts (keep these working — do not break)

These are not experiments; they verify the rig before a real run.

- `test_connection.py` — can we connect to the (simulated) drone?
- `takeoff_test.py` — single-drone takeoff/land.
- `three_takeoff_test.py` — three-drone takeoff/land.
- `repeat_test.py` — repeatability / repeated-run check. `[CONFIRM purpose]`

Run the relevant sanity check before every experiment session.

---

## 7. Data logging standard (makes results analysis-ready)

Consistency here is what lets one analysis script read every experiment.

**File naming:** `data/raw/<experiment>_<YYYYMMDD-HHMMSS>_run<NN>.csv`
Each CSV has a sidecar `…​.json` with the run's metadata (see below).

**Metadata sidecar (per run):**
```json
{
  "experiment": "surprise_minimization",
  "timestamp": "2026-10-06T14:00:00",
  "mode": "sim",                     // "sim" or "real"
  "n_drones": 1,
  "git_commit": "<hash>",
  "params": { "target_distance": 0.5, "noise": "low" },
  "seed": 42,
  "notes": ""
}
```

**Core column schema (every experiment, one row per drone per timestep):**

| Column | Unit | Meaning |
|---|---|---|
| `t` | s | time since takeoff |
| `drone_id` | — | which drone (0 for single) |
| `x, y, z` | m | estimated position |
| `vx, vy, vz` | m/s | estimated velocity |
| `cmd_type` | — | setpoint type sent (vel/pos/hl) |
| `cmd_x, cmd_y, cmd_z` | m or m/s | commanded setpoint |
| `battery_v` | V | battery voltage (real); sim may be constant |

Experiments **add** their own columns on top (listed in each section above).
Never rename core columns between experiments.

**Rules:**
- One run = one CSV. Never append two runs into one file.
- Log at a fixed rate; record that rate in the metadata.
- Write the metadata sidecar even for throwaway runs.

---

## 8. Analysis plan

- **Per-run metrics:** `[FILL IN — e.g. RMS distance error, mean & peak surprise,
  control effort = Σ|action|, settling time]`
- **Aggregation:** combine the repetitions of each condition → mean ± std (or
  confidence interval) per condition.
- **Core comparison:** proposed (E1) vs baseline (E2) on the same DV, same
  conditions. `[CONFIRM this is the main result]`
- **Standard plots:**
  - surprise over time (per run and averaged);
  - measured vs target distance over time;
  - DV vs independent variable (e.g. RMS error vs noise level), baseline vs
    proposed on the same axes.
- **Statistics (optional but recommended):** `[FILL IN — e.g. paired test across
  seeds; report effect size]`
- All analysis reads only `data/raw/` + sidecars, and writes figures to
  `analysis/figures/`. Analysis never re-runs flights.

---

## 9. Deliverables & definition of done

- [ ] Every experiment in Section 5 produces CSVs in the core schema.
- [ ] `analysis/` reproduces the standard plots from `data/raw/` with one command.
- [ ] The core comparison (E1 vs E2) is plotted and its success criterion is
      evaluated.
- [ ] Each experiment script runs in sim **and** has a documented path to real
      hardware (see `README.md` → Simulation → Real drone).

---

## 10. Open questions to confirm (fill these first)

1. Is the framework **active inference / surprise minimisation**, and what is the
   exact **hypothesis** (Section 1)?
2. What **physically defines surprise** — expected vs observed *what*? (Section 2)
3. Is `distance_controller.py` the **baseline** that the surprise-minimising
   method is compared against? (Section 1 / E2)
4. Which **decks/sensors** are used (Multiranger? Flow deck?) (Section 3)
5. What are the **default run parameters** (altitude, duration, repetitions,
   rates) (Section 4)?
