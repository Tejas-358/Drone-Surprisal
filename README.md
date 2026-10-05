# Drone-Surprisal

Surprise-minimisation experiments for the Crazyflie nano quadrotor, run in the
**CrazySim** software-in-the-loop (SITL) simulator and designed so the **same
code runs on real Crazyflies**.

> **What is this project?** See **[EXPERIMENTS.md](EXPERIMENTS.md)** for the
> research framework, hypotheses, and the detailed spec of every experiment.
> This README covers how the repository is organised and how to run things.

---

## Repository structure

The layout below is the **target structure**. The current repo has all scripts
at the top level; moving them into this structure is step one (see *Migration*
at the bottom). Keeping this structure is what makes results analysis-ready and
the code portable to hardware.

```
drone-surprisal/
├── README.md                  # this file — how to run things
├── EXPERIMENTS.md             # what we test and why (source of truth)
├── requirements.txt           # Python dependencies
├── config/
│   ├── default.yaml           # shared defaults (altitude, rates, log dir, URI)
│   └── experiments/           # one file per experiment: the params to vary
│       ├── surprise_minimization.yaml
│       ├── distance.yaml
│       ├── wall_follow.yaml
│       └── formation.yaml
├── src/
│   ├── controllers/           # the control logic (sim == real)
│   │   ├── base_controller.py
│   │   ├── distance_controller.py
│   │   └── surprise_minimization.py
│   ├── experiments/           # orchestration: setup → run → log
│   │   ├── surprise_minimization_experiment.py
│   │   ├── distance_experiment.py
│   │   ├── wall_follow_experiment.py
│   │   └── formation_experiment.py
│   ├── utils/                 # shared helpers — the key to sim-to-real
│   │   ├── connection.py      # build URI (sim vs real), connect/disconnect
│   │   ├── flight.py          # takeoff, land, safe emergency shutdown
│   │   ├── logging.py         # standard CSV logger + metadata sidecar
│   │   └── sensors.py         # read Multiranger / Flow / state estimate
│   └── tests/                 # sanity checks, NOT experiments
│       ├── test_connection.py
│       ├── takeoff_test.py
│       ├── three_takeoff_test.py
│       └── repeat_test.py
├── scripts/
│   └── run_experiment.sh      # launch an experiment by name + config
├── data/
│   └── raw/                   # CSV logs (one per run) + .json sidecars
├── analysis/
│   ├── load.py                # load + concatenate runs into a dataframe
│   ├── metrics.py             # per-run metrics (RMS error, mean surprise, …)
│   ├── plots.py               # standard figures → analysis/figures/
│   └── figures/
└── cache/                     # simulator / build cache (git-ignored)
```

---

## Installation

### 1. Prerequisite: CrazySim

Install and build CrazySim (the SITL simulator) by following its own repository.
It brings in the Crazyflie firmware (SITL build), `crazyflie-lib-python`, and
Gazebo. This project assumes you can already start the simulator and see a drone.

### 2. Python environment for this repo

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

`requirements.txt` should pin at least:

```
cflib            # crazyflie-lib-python
numpy
pandas
pyyaml
matplotlib
```
`[CONFIRM exact versions once the environment is frozen: pip freeze]`

---

## Running an experiment

1. Start the CrazySim simulator (separate terminal) and spawn the number of
   drones the experiment needs.
2. Run an experiment by name; parameters come from its config file:

```bash
# via the runner
./scripts/run_experiment.sh surprise_minimization

# or directly
python -m src.experiments.surprise_minimization_experiment \
       --config config/experiments/surprise_minimization.yaml \
       --mode sim --runs 10
```

Every run writes `data/raw/<experiment>_<timestamp>_run<NN>.csv` plus a matching
`.json` metadata sidecar. **Do not** hand-edit these files.

---

## Configuration

All tunable values live in `config/`, never hard-coded in scripts. This is what
lets you re-run an experiment with different parameters without editing code, and
what records exactly what was run.

- `config/default.yaml` — shared: altitude, control rate, log rate, log dir,
  connection settings, default seed.
- `config/experiments/<name>.yaml` — per-experiment: the independent variables
  and their values (e.g. target distances, noise levels, formation spacing).

Rule: if a number affects a result, it belongs in a config file and in the run's
metadata sidecar — not buried in the script.

---

## Writing a new experiment (conventions)

Follow these so every experiment's output is comparable and auto-analysable:

1. **Reuse the helpers.** Connection, takeoff, landing, and logging come from
   `src/utils/` — do not re-implement them per script.
2. **Separate controller from experiment.** The *controller* (`src/controllers/`)
   decides actions; the *experiment* (`src/experiments/`) sets up conditions,
   runs repetitions, and logs. This keeps controllers reusable on hardware.
3. **Log the core schema** (see EXPERIMENTS.md §7) plus your experiment's extra
   columns. Never rename core columns.
4. **One run = one CSV + one sidecar.** Loop repetitions inside the experiment.
5. **No sim-only logic in controllers.** Anything simulator-specific goes behind
   `src/utils/connection.py` or config.
6. **Fail safe.** Always wrap a run so that on any error or Ctrl-C the drone
   lands / motors stop (`src/utils/flight.py`).

---

## Data & analysis

- Raw data is immutable: `data/raw/` is append-only, one file per run.
- `analysis/load.py` reads all runs of an experiment (using the sidecars to tag
  each row with its parameters) into a single dataframe.
- `analysis/metrics.py` computes per-run metrics; `analysis/plots.py` produces
  the standard figures defined in EXPERIMENTS.md §8.
- Analysis **never** flies the drone — it only reads `data/raw/`.

```bash
python -m analysis.plots --experiment surprise_minimization
# figures land in analysis/figures/
```

---

## Simulation → real drone (portability guide)

The whole point of this layout is that moving to hardware changes *configuration*,
not control logic. Keep to these rules:

1. **One switch:** `--mode sim|real` (or `mode:` in config). Nothing else in the
   control path knows which it is.
2. **Connection is the only place that differs.** `src/utils/connection.py`
   builds the link URI from the mode:
   - **sim:** CrazySim exposes each SITL drone to CFLib on a local port in the
     `19850+N` range (drone *N*). `[CONFIRM the exact URI form your CrazySim
     build uses, e.g. udp://… .]`
   - **real:** a radio URI such as `radio://0/80/2M/E7E7E7E7E7`.
3. **Identical controllers.** `src/controllers/` must not import anything
   sim-specific. If it runs in sim, it should run on hardware with the same code.
4. **Safety wrappers for hardware (configurable, on by default on real):**
   - battery check before arming and a low-voltage auto-land;
   - a geofence / max-altitude / max-speed clamp;
   - a watchdog timeout that lands if no command is sent;
   - emergency-stop on exception or Ctrl-C.
5. **Sensors:** confirm the same decks are present on the real drone as were
   simulated (Multiranger, Flow deck) `[CONFIRM]`.
6. **Start conservative on hardware:** low altitude, low speed, one drone, open
   space, a hand on the kill switch, then scale up.

> Golden rule: if you ever need an `if sim:` inside a controller, stop — that
> logic belongs in `utils/connection.py` or in config instead.

---

## Testing / sanity checks

Before any experiment session:

```bash
python -m src.tests.test_connection      # link works
python -m src.tests.takeoff_test         # single drone takeoff/land
python -m src.tests.three_takeoff_test   # multi-drone (if used)
```

---

## Migration from the current flat layout

The repo currently has every script at the top level. To adopt this structure
without breaking the working flight:

1. Commit the current working state to a branch first (a known-good baseline).
2. Move files into `src/…` per the tree above; update imports.
3. Extract repeated connect/takeoff/land/log code into `src/utils/`.
4. Pull hard-coded numbers into `config/`.
5. Re-run the sanity checks after each step — never move everything at once.

See **[EXPERIMENTS.md](EXPERIMENTS.md)** for what each experiment must do.
