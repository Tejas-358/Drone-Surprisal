# Drone-Surprisal

Surprise-minimisation experiments with Crazyflie nano-drones. Every experiment
runs **in the CrazySim simulator and on real Crazyflies with the same code**;
only `--mode sim` / `--mode real` changes.

> **What we test and why:** see **[EXPERIMENTS.md](EXPERIMENTS.md)** (hypotheses,
> definitions, maths, metrics). This README is the *how*: setup, running,
> data and analysis.

---

## The experiments at a glance

| Experiment | Drones | Command name | Controllers compared |
|---|---|---|---|
| **Two-drone surprisal distance**: keep 0.6 m apart while moving to a goal | 2 | `surprisal_goal_navigation` | `fixed` (baseline) vs `adaptive` (gain grows with surprise) |
| **Three drones to a target**: fly together to one point, no leader | 3 | `target_swarm` | `goal_only` (baseline) vs `surprise` (target + neighbours) |
| **Convoy E2**: leader–follower train, leader stops at a virtual wall | 3 | `convoy_baseline` | baseline |
| **Convoy E1**: same, followers slow down when surprised | 3 | `surprise_minimization` | surprise-gated (sweep `noise_scale`) |
| **Formation E3**: fix a wonky line of drones | 3 | `formation_distance` | — |

Sanity checks (not experiments): `test_connection`, `takeoff_test`,
`three_takeoff_test`, `hover_telemetry_test`, `lighthouse_diagnostic`,
`controller_diagnostic`, `coordinate_navigation_test`.

---

## Repository layout

```
Drone-Surprisal/
├── README.md, EXPERIMENTS.md, requirements.txt
├── config/
│   ├── default.yaml                 # shared: mode, URIs, height, arena, SAFETY limits
│   └── experiments/                 # one file per experiment (all its numbers)
│       ├── surprisal_goal_navigation.yaml
│       ├── target_swarm.yaml
│       ├── convoy.yaml              # sim (large)        convoy_real.yaml  # real arena
│       └── formation_distance.yaml
├── src/
│   ├── controllers/                 # pure maths, identical for sim and real
│   │   ├── convoy_controller.py, formation_controller.py, target_swarm_controller.py
│   ├── experiments/                 # one script per experiment: fly, control, log
│   ├── utils/
│   │   ├── connection.py            # load config, sim/real URIs, connect
│   │   ├── flight.py                # arm, take off, fly_to_positions, land, fly_safely()
│   │   ├── safety.py                # pre-flight + in-flight checks, emergency stop
│   │   ├── sensors.py               # position/velocity/battery telemetry
│   │   ├── observation.py           # optional noise on what the controller sees
│   │   ├── cli.py                   # shared command-line options
│   │   └── run_log.py               # save each run: CSV + JSON sidecar
│   ├── simulation/                  # standalone point-mass simulation (no CrazySim)
│   └── tests/                       # sanity checks
├── scripts/
│   ├── run_experiment.sh            # start CrazySim, run ONE experiment, stop CrazySim
│   └── run_all_sim.sh               # run EVERY experiment N times + all analysis
├── analysis/                        # read data/raw, make figures + tables (never flies)
│   ├── load_runs.py
│   ├── plot_goal_navigation_runs.py, plot_target_swarm_runs.py,
│   ├── plot_convoy_runs.py, plot_formation_runs.py, plot_surprisal_distance.py
│   └── figures/                     # output PNGs
└── data/
    ├── raw/                         # one CSV + one JSON per run (never hand-edited)
    └── simulation/                  # outputs of src/simulation
```

---

## Setup

### A. Server / Linux machine (simulator)

1. Install and build **CrazySim** (follow its own repository). It provides the
   Crazyflie SITL firmware, Gazebo and `cflib`.
2. Clone this repo and install the Python packages into CrazySim's venv:
   ```bash
   git clone https://github.com/Tejas-358/Drone-Surprisal.git
   cd Drone-Surprisal
   source ~/crazysim-venv/bin/activate      # your CrazySim venv
   pip install -r requirements.txt
   ```
3. Tell the scripts where CrazySim is (defaults shown):
   ```bash
   export CRAZYSIM_DIR=$HOME/CrazySim/crazyflie-firmware
   export VENV=$HOME/crazysim-venv
   ```

### B. Laptop (real drones, Windows or Linux)

1. `python -m venv venv`, activate it (`venv\Scripts\activate` on Windows),
   `pip install -r requirements.txt`.
2. Crazyradio with the libusb driver (Windows: Zadig). **Close cfclient** before
   running any script — only one program can use the radio.
3. Real-drone addresses are in `config/default.yaml` → `uris: real:`.
4. Every drone needs: up-to-date firmware, a Lighthouse deck, and the **same**
   base-station geometry (set it up once in cfclient, Export, then Import on every
   drone). Check with `python -m src.tests.lighthouse_diagnostic --mode real --drones 3`.

---

## Running in the simulator (server)

### Everything at once

```bash
./scripts/run_all_sim.sh                       # all experiments, 3 repeats each, then analysis
REPEATS=1 ./scripts/run_all_sim.sh             # quick check: one run of everything
ONLY=target_swarm REPEATS=5 ./scripts/run_all_sim.sh   # one experiment only
```

Each run starts a fresh CrazySim and stops it afterwards. Options:
`REPEATS`, `NOISE` (default 0.03), `SEED_BASE`, `N_SWEEP` (convoy N values),
`ONLY`. Failed runs are listed at the end. The full default batch takes about 1 h.

### One run

```bash
N_DRONES=2 ./scripts/run_experiment.sh surprisal_goal_navigation --mode sim --controller adaptive --scenario too_far
N_DRONES=3 ./scripts/run_experiment.sh target_swarm --mode sim --controller surprise --noise 0.03 --seed 1
N_DRONES=3 ./scripts/run_experiment.sh surprise_minimization --mode sim --set noise_scale=0.2
```

Or, with CrazySim already running in another terminal, call Python directly:
```bash
python -m src.experiments.target_swarm_experiment --mode sim --controller goal_only
```

**Why `--noise`?** The simulator is perfectly repeatable, so repeated runs would
be identical. `--noise 0.03` adds Gaussian noise (std 3 cm, 3 cm/s) to what the
controller *sees*, like real Lighthouse jitter; `--seed` makes each noisy run
repeatable. Safety checks always use the true state.

**Start positions in sim:** CrazySim spawns drones at fixed spots, so after
takeoff every experiment flies its drones to the start positions from its config
file — each drone at its own height on the way, so paths cannot collide.

---

## Running on real drones (laptop)

Before every session (cfclient closed, charged batteries ≥ 3.7 V):
```bash
python -m src.tests.test_connection --mode real --drones 3     # positions sensible, z ≈ 0
python -m src.tests.three_takeoff_test --mode real            # all three take off and land
```
Then, for example:
```bash
python -m src.experiments.surprisal_goal_navigation_experiment --mode real --controller fixed
python -m src.experiments.target_swarm_experiment --mode real --controller surprise
python -m src.experiments.convoy_baseline_experiment --mode real --config config/experiments/convoy_real.yaml
```
On real drones the two-drone experiment can start where you placed the drones
(`--scenario as_placed`, the default) or fly to a scenario's start positions
(`--scenario too_far` / `too_close`).

---

## Command-line options (every experiment)

| Option | Meaning |
|---|---|
| `--mode sim\|real` | simulator or real drones (overrides `mode` in `default.yaml`) |
| `--config FILE` | use another experiment YAML |
| `--noise STD` | noise on what the controller sees (m, m/s), default 0 |
| `--seed N` | random seed for that noise (random if omitted; always saved) |
| `--set key=value` | change one config value for this run (repeatable), e.g. `--set duration=20` |
| `--controller …` | which controller (two-drone and target-swarm experiments) |
| `--scenario …` | two-drone experiment only: `too_far`, `too_close`, `as_placed` |

---

## Data

Every run writes two files to `data/raw/`:

```
<experiment>_<controller>_<mode>_<YYYYMMDD-HHMMSS>.csv    one row per control step (10 Hz)
<experiment>_<controller>_<mode>_<YYYYMMDD-HHMMSS>.json   how it was run + its result metrics
```

The JSON holds: experiment, controller, mode, scenario, noise, seed, `aborted`,
seconds flown, git commit, the complete config, and the metrics. **Aborted runs
are kept but excluded from analysis** (`"aborted": true`). The two-drone
experiment also appends one row per run to
`data/raw/surprisal_goal_navigation_summary.csv`.

Raw data is never edited by hand.

---

## Analysis

```bash
python -m analysis.plot_goal_navigation_runs --mode sim     # or --mode real
python -m analysis.plot_target_swarm_runs --mode sim
python -m analysis.plot_convoy_runs --mode sim
python -m analysis.plot_formation_runs --mode sim
python -m analysis.plot_surprisal_distance data/raw/<one run>.csv --target 0.6
```

Each script uses only **finished** runs of the chosen mode, saves a figure to
`analysis/figures/<experiment>_runs_<mode>.png` and prints a table of metric
**mean ± std** per controller (and scenario / N). Add `--show` to open a window.
What each metric means is listed in EXPERIMENTS.md.

---

## Safety (built into every flight)

All flights go through `fly_safely()` (`src/utils/flight.py`):

- **Before arming** (`safety.preflight_check`): Lighthouse deck present, battery
  ≥ `min_battery_v`, position steady (jitter ≤ 5 cm) and on the floor, inside the
  arena; for experiments with fixed start spots, each drone on its spot.
  *(Real drones only for deck/battery/spots — the simulator has none.)*
- **While flying** (`safety.check_in_flight`, 10× per second): inside the arena,
  slower than `max_speed`, data fresher than `max_data_age`.
  A problem must persist 3 checks (0.3 s) before it counts, so one Lighthouse glitch
  does not drop a drone.
  - stale data / drones too close → **land all**
  - a drone outside the arena or flying away → **cut that drone's motors**, land the others
- **Always** (normal end, error, Ctrl-C): land, wait for touchdown, motors off, disarm.

All limits are in `config/default.yaml`. Experiment files must not reuse those
key names — `load_config()` prints a NOTE if an experiment overrides a default.

---

## Writing a new experiment

1. Maths in `src/controllers/<name>_controller.py` — positions in, velocities out,
   no drone access, no `if sim:`.
2. All numbers in `config/experiments/<name>.yaml`.
3. Script in `src/experiments/<name>_experiment.py`: parser from `cli.make_parser`,
   a `mission(drones, config, states)` passed to `fly_safely()`, call
   `check_in_flight()`/`watch()` every step, use `Observer` for what the controller
   sees, save with `run_log.save_run()` in a `finally:` block.
4. A plot script in `analysis/` using `load_runs()`.
5. Describe it in EXPERIMENTS.md.

---

## Troubleshooting (things we have hit)

| Symptom | Cause / fix |
|---|---|
| `Too many packets lost` | cfclient open, weak battery, Crazyradio too close to the laptop (use a USB extension) |
| Script hangs on "Connecting…", Ctrl-C does nothing | cfclient holds the radio. Close it, kill Python, re-plug the radio |
| `z=-0.2` / `z=108` on the floor, "position jumps" | Lighthouse not tracking: run `lighthouse_diagnostic`; all drones must show the **same** geometry and receive both base stations |
| `refused to arm … LOCKED/CRASHED` | the drone crashed earlier — power-cycle it |
| `did not take off … ARMED, can fly` | weak battery, or (old code) needed a reset between runs — fixed by `send_notify_setpoint_stop()` before takeoff |
| A drone falls when two get close | one hid a base station from the other; place drones along x, not y |
| `ABORT: no fresh data` | radio link dropped (often a low battery); the code lands everything |
| `.sh` fails with `$'\r'` on Linux | Windows line endings; `.gitattributes` keeps `.sh` as LF — re-clone or `dos2unix` |
