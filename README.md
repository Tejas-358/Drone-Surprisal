# Drone-Surprisal

Surprise-minimisation experiments for a three-drone Crazyflie **leader–follower
convoy**, run in the **CrazySim** software-in-the-loop (SITL) simulator and
designed so the **same code runs on real Crazyflies**.

> **What is this project?** See **[EXPERIMENTS.md](EXPERIMENTS.md)** for the
> research framework, hypotheses, definitions and the spec of every experiment.
> This README covers how the repository is organised and how to run things.

---

## Repository structure

Items marked *(planned)* are specified but not written yet.

```
Drone-Surprisal/
├── README.md                  # this file — how to run things
├── EXPERIMENTS.md             # what we test and why (source of truth)
├── requirements.txt           # Python dependencies
├── config/
│   ├── default.yaml           # shared: mode, URIs, height, rates, log dir
│   └── experiments/
│       ├── convoy.yaml              # E1 + E2 (one shared file → same scenario)
│       └── formation_distance.yaml  # E3
├── src/
│   ├── controllers/           # pure control maths (sim == real)
│   │   ├── convoy_controller.py     # leader, follower, surprise gating, repulsion
│   │   └── formation_controller.py  # formation-potential gradient descent
│   ├── experiments/           # orchestration: setup → run → log
│   │   ├── surprise_minimization_experiment.py  # E1 (proposed)
│   │   ├── convoy_baseline_experiment.py        # E2 (baseline)
│   │   └── formation_distance_experiment.py     # E3
│   ├── utils/                 # shared helpers — the key to sim-to-real
│   │   ├── connection.py      # load config, build URIs (sim/real), connect
│   │   ├── flight.py          # takeoff, safe landing, fly_safely() wrapper
│   │   ├── sensors.py         # stream stateEstimate (x, y, z, vx, vy, vz)
│   │   └── logging.py         # (planned) standard CSV logger + JSON sidecar
│   └── tests/                 # sanity checks, NOT experiments
│       ├── test_connection.py
│       ├── takeoff_test.py
│       ├── three_takeoff_test.py
│       └── hover_telemetry_test.py
├── scripts/
│   └── run_experiment.sh      # start CrazySim, run one experiment, stop CrazySim
├── data/
│   └── raw/                   # one CSV per run (+ .json sidecar, planned)
│       └── legacy/            # CSVs from the old scripts — unchanged
├── analysis/                  # (planned) load, metrics, plots → analysis/figures/
├── legacy/                    # the original scripts, unchanged, for reference
└── cache/                     # CFLib parameter cache (git-ignored)
```

---

## Installation

### 1. Prerequisite: CrazySim

Install and build CrazySim by following its own repository. It brings in the
Crazyflie SITL firmware, `crazyflie-lib-python` (cflib) and Gazebo. This project
assumes you can already start the simulator and see a drone.

### 2. Python environment for this repo

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

If CrazySim installed its own cflib into another venv, you can use that venv
instead (see `VENV` below).

---

## Running

All commands are run from the **repo root**.

### Option A — one command (starts and stops CrazySim for you)

```bash
./scripts/run_experiment.sh convoy_baseline
./scripts/run_experiment.sh surprise_minimization
./scripts/run_experiment.sh formation_distance --config my_settings.yaml
```

Paths are set with environment variables:

| Variable | Default | Meaning |
|---|---|---|
| `CRAZYSIM_DIR` | `<repo>/../CrazySim/crazyflie-firmware` | CrazySim firmware folder |
| `VENV` | `<repo>/.venv` (skipped if missing) | Python venv to activate |
| `N_DRONES` | `3` | drones to spawn |
| `SIM_STARTUP_WAIT` | `10` | seconds to wait for CrazySim |

```bash
CRAZYSIM_DIR=~/crazyflie/CrazySim/crazyflie-firmware \
VENV=~/crazyflie/crazysim-venv \
./scripts/run_experiment.sh convoy_baseline
```

### Option B — CrazySim already running in another terminal

```bash
python -m src.experiments.convoy_baseline_experiment
python -m src.experiments.surprise_minimization_experiment
python -m src.experiments.formation_distance_experiment
```

Every experiment accepts `--config <file>` (default: its file in
`config/experiments/`) and `--mode sim|real` (overrides `mode` in
`config/default.yaml`).

Each run currently writes `data/raw/<experiment>_<timestamp>.csv`.
**Do not** hand-edit these files.

---

## Configuration

All tunable values live in `config/`, never hard-coded in scripts.

- `config/default.yaml` — shared: `mode`, URIs per mode, height, takeoff/land
  time, telemetry rate, log dir.
- `config/experiments/<name>.yaml` — the experiment's scenario and controller
  values. Loaded on top of `default.yaml` (experiment values win).

To try different values, copy the experiment's YAML, change it, and pass it
with `--config`. Rule: if a number affects a result, it belongs in a config file.

---

## Writing a new experiment (conventions)

1. **Reuse the helpers.** Connect, take off, land and stream telemetry with
   `src/utils/` — do not re-implement them.
2. **Separate controller from experiment.** The *controller*
   (`src/controllers/`) is pure maths: numbers in, commands out. It never talks
   to a drone. The *experiment* (`src/experiments/`) reads telemetry, calls the
   controller, sends commands and logs.
3. **Fly through `fly_safely()`.** Write your experiment as a
   `mission(drones, config)` function and pass it to `fly_safely()`, which
   connects, takes off, and **always lands** — on normal finish, error or Ctrl-C.
4. **Log the core schema** (EXPERIMENTS.md §7) plus your own columns. One run =
   one CSV.
5. **No sim-only logic in controllers.** Anything that differs between sim and
   real goes in `src/utils/connection.py` or config.

---

## Simulation → real drone

Moving to hardware changes *configuration*, not control logic.

1. **One switch:** `mode: sim|real` in `config/default.yaml`, or `--mode` on
   the command line. Nothing in the control path knows which it is.
2. **Connection is the only place that differs.** `src/utils/connection.py`
   picks the URIs for the mode:
   - **sim:** `udp://127.0.0.1:1985N` for drone N (19850 = CF1);
   - **real:** radio URIs such as `radio://0/80/2M/E7E7E7E7E1` — set yours in
     `config/default.yaml` *(still to fill in)*.
3. **Positioning on hardware = Lighthouse.** All controllers use only the
   absolute `stateEstimate` position and velocity (x, vx), which Lighthouse
   provides directly. **No Multiranger or Flow deck is needed**, and the
   controllers run **unchanged**.
4. **Identical controllers.** `src/controllers/` must not import anything
   sim-specific. If you ever need an `if sim:` inside a controller, stop —
   that logic belongs in `connection.py` or config.
5. **Safety already in place:** `fly_safely()` lands on any error or Ctrl-C;
   landing calls `send_notify_setpoint_stop()` first (required on real
   firmware after velocity setpoints); motors are stopped after landing; the
   convoy followers have a safety repulsion that surprise gating cannot cancel.
6. **Still to add before hardware (planned):** battery check and low-voltage
   auto-land; geofence / max-altitude / max-speed clamp; command watchdog.
7. **Start conservative:** low altitude, low speed, one drone, open space, a
   hand on the kill switch — then scale up.

---

## Sanity checks

Run before every experiment session (CrazySim running, from the repo root):

```bash
python -m src.tests.test_connection         # link + telemetry, never flies
python -m src.tests.takeoff_test            # CF1 takeoff / hover / land
python -m src.tests.three_takeoff_test      # 3 drones takeoff / hover / land
python -m src.tests.hover_telemetry_test    # CF1 hover, position + height check
```

---

## Data & analysis

- `data/raw/` is append-only: one file per run, never edited.
- `data/raw/legacy/` holds the CSVs from the pre-migration scripts, unchanged.
- *(planned)* `analysis/` will load all runs, compute the per-run metrics
  (EXPERIMENTS.md §8) and draw the standard figures into `analysis/figures/`.
  Analysis never flies a drone.

---

## Legacy scripts

The original flat-layout scripts are kept unchanged in `legacy/` as a
known-good reference (see `legacy/README.md` for where each one moved). Each is
deleted once its new version has been confirmed in the simulator.
