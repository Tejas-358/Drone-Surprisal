"""
target_swarm_experiment.py — three drones fly TOGETHER to one target point,
each moving to lower its own surprise about the target and its neighbours.

What happens in one run:
  1. Connect to 3 drones, pre-flight check, take off (done by fly_safely).
  2. Fly to the start positions (each drone at its own height on the way).
  3. For `duration` seconds, 10x per second: every drone computes its total
     surprise F = S_goal + sum S_neighbours and moves down its gradient
     (see src/controllers/target_swarm_controller.py). With
     --controller goal_only the drones ignore each other (baseline).
  4. Land (fly_safely). Save CSV + JSON (also after an abort) with metrics.

Expected behaviour:
  surprise  — the drones head for the target and settle in a triangle
              around it, about desired_distance apart.
  goal_only — the drones crowd towards the same point; only the safety
              push keeps them apart (smaller spacing, more safety pushes).

Run from the repo root:
    python -m src.experiments.target_swarm_experiment --mode sim --controller surprise
    python -m src.experiments.target_swarm_experiment --mode sim --controller goal_only
    extra: --noise 0.03 --seed 1   (mimic Lighthouse jitter in sim)
"""

import warnings

# Hide only the known cflib message about the legacy velocity setpoint.
warnings.filterwarnings(
    "ignore",
    message="Using legacy TYPE_VELOCITY_WORLD_LEGACY.*",
    category=DeprecationWarning,
)

import itertools
import math
import time

from src.utils import cli
from src.utils.flight import fly_safely, fly_to_positions
from src.utils.observation import Observer
from src.utils.run_log import save_run
from src.utils.safety import watch
from src.controllers.target_swarm_controller import swarm_step


DEFAULT_CONFIG = "config/experiments/target_swarm.yaml"
N_DRONES = 3


def clamp(value, low, high):
    """Keep value inside [low, high]."""
    return max(low, min(high, value))


def pair_distances(points):
    """
    Horizontal distance for every pair of drones.
    Input:  points — list of (x, y)
    Output: dict {"d12": ..., "d13": ..., "d23": ...}
    """
    return {f"d{i + 1}{j + 1}": math.hypot(points[i][0] - points[j][0],
                                           points[i][1] - points[j][1])
            for i, j in itertools.combinations(range(len(points)), 2)}


def run_mission(drones, config, states, variant, observer, rows):
    """
    The control loop (see the top of this file).

    Input:  drones   — 3 connected, hovering drones (from fly_safely)
            config   — default.yaml + target_swarm.yaml
            states   — live TRUE telemetry (states[0] = CF1, ...)
            variant  — "surprise" or "goal_only"
            observer — adds the optional --noise to what the controller sees
            rows     — empty list; one dictionary per step is appended
    Output: none.

    Called from: fly_safely(), via main().
    """
    fly_to_positions(drones, states, config, config["start_positions"])
    target = config["target"]
    height = config["height"]
    print(f"\nController: {variant}   target: {target}   "
          f"desired spacing: {config['desired_distance']} m")

    start = time.time()
    try:
        while time.time() - start < config["duration"]:
            step_start = time.time()
            t = step_start - start

            # --- Safety on the TRUE positions
            true_xy = [(s["x"], s["y"]) for s in states]
            dists = pair_distances(true_xy)
            if min(dists.values()) < config["critical_distance"]:
                raise RuntimeError(f"ABORT: two drones only {min(dists.values()):.2f} m apart")
            for i, s in enumerate(states):
                if not config["min_altitude"] <= s["z"] <= config["max_altitude"]:
                    raise RuntimeError(f"ABORT: CF{i + 1} at z={s['z']:.2f} m")

            # --- Controller: what the drones see (noisy copies with --noise)
            seen = [observer.see(s) for s in states]
            commands = swarm_step([(o["x"], o["y"]) for o in seen], target, config, variant)

            for scf, o, c in zip(drones, seen, commands):
                vz = clamp(config["altitude_kp"] * (height - o["z"]),
                           -config["max_vertical_speed"], config["max_vertical_speed"])
                scf.cf.commander.send_velocity_world_setpoint(c["vx"], c["vy"], vz, 0.0)

            # --- Log TRUE positions (what happened) + what each drone decided
            cx = sum(p[0] for p in true_xy) / N_DRONES
            cy = sum(p[1] for p in true_xy) / N_DRONES
            row = {"t": round(t, 3), "variant": variant,
                   "centre_error": math.hypot(cx - target[0], cy - target[1])}
            for i, (s, c) in enumerate(zip(states, commands)):
                n = i + 1
                row.update({f"x{n}": s["x"], f"y{n}": s["y"], f"z{n}": s["z"],
                            f"vx{n}": c["vx"], f"vy{n}": c["vy"],
                            f"S_goal{n}": c["S_goal"], f"S_neigh{n}": c["S_neighbours"],
                            f"F{n}": c["F"], f"protect{n}": c["protective"]})
            row.update(dists)
            row["F_total"] = sum(c["S_goal"] + c["S_neighbours"] for c in commands)
            rows.append(row)

            if len(rows) % 10 == 1:   # about once per second
                print(f"t={t:5.1f}s  centre_err={row['centre_error']:.2f} m  "
                      f"d12={dists['d12']:.2f} d13={dists['d13']:.2f} d23={dists['d23']:.2f}  "
                      f"F_total={row['F_total']:7.2f}"
                      f"{'  PROTECT' if any(c['protective'] for c in commands) else ''}")

            # Safety checks (arena, fly-away, stale data) during the rest
            # of this control period.
            watch(states, config, max(0.0, config["control_period"] - (time.time() - step_start)))

    finally:
        for scf in drones:   # stop moving before fly_safely() lands them
            try:
                scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass


def metrics(rows, config):
    """
    Per-run results (distances in m, times in s):
      arrival_time         — first time the swarm centre is within
                             arrival_tolerance of the target (empty if never)
      final_centre_error   — distance of the swarm centre from the target at the end
      min_pair_distance    — closest any two drones ever came (safety)
      final_spacing_error  — mean |d_ij - desired_distance| over the 3 pairs, at the end
      final_spacing_spread — max minus min of the 3 pair distances at the end
                             (0 = perfectly even triangle)
      final_total_surprise — sum of everyone's surprise (target + neighbours),
                             averaged over the last 5 s
      protective_fraction  — share of drone-steps with the safety push active
      control_effort       — sum of |commanded speed| x dt over all drones
    """
    target_d = config["desired_distance"]
    dt = config["control_period"]
    pairs = ["d12", "d13", "d23"]

    arrival = next((round(r["t"], 2) for r in rows
                    if r["centre_error"] <= config["arrival_tolerance"]), "")
    last = rows[-1]
    tail = [r for r in rows if r["t"] >= last["t"] - 5.0]

    return {
        "arrival_time": arrival,
        "final_centre_error": round(last["centre_error"], 3),
        "min_pair_distance": round(min(min(r[p] for p in pairs) for r in rows), 3),
        "final_spacing_error": round(sum(abs(last[p] - target_d) for p in pairs) / 3, 3),
        "final_spacing_spread": round(max(last[p] for p in pairs) - min(last[p] for p in pairs), 3),
        "final_total_surprise": round(sum(r["F_total"] for r in tail) / len(tail), 3),
        "protective_fraction": round(
            sum(r[f"protect{n}"] for r in rows for n in (1, 2, 3)) / (3 * len(rows)), 3),
        "control_effort": round(sum(math.hypot(r[f"vx{n}"], r[f"vy{n}"]) * dt
                                    for r in rows for n in (1, 2, 3)), 3),
    }


def main():
    parser = cli.make_parser(DEFAULT_CONFIG, "Three drones fly to a target by surprise minimisation")
    parser.add_argument("--controller", choices=["surprise", "goal_only"], required=True,
                        help="surprise = target + neighbours (proposed); "
                             "goal_only = target only (baseline)")
    args = parser.parse_args()
    config, seed = cli.load(args)
    observer = Observer(args.noise, seed)

    rows = []
    aborted = True   # stays True unless the flight finishes normally
    try:
        fly_safely(config, N_DRONES,
                   lambda drones, cfg, states: run_mission(drones, cfg, states, args.controller,
                                                           observer, rows))
        aborted = False
    finally:
        if rows:
            result = metrics(rows, config)
            save_run(rows, config, "target_swarm", args.controller,
                     {"noise_std": args.noise, "seed": seed, "aborted": aborted,
                      "flown_s": rows[-1]["t"], "metrics": result})
            for key, value in result.items():
                print(f"  {key:22s} {value}")
            if aborted:
                print("NOTE: this run was ABORTED — exclude it when comparing.")
        else:
            print("No data recorded.")


if __name__ == "__main__":
    main()
