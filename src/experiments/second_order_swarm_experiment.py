"""
second_order_swarm_experiment.py — three drones fly TOGETHER to one target,
each ACCELERATING down its own surprise (target + neighbours) with one
damper per spring. EXPERIMENTS.md §8.

What happens in one run:
  1. Connect to 3 drones, take off (fly_safely), fly to the start positions.
  2. For `duration` seconds, 10x per second, every drone:
       - computes its acceleration  a_i = -kappa grad F_i - damping
         (src/controllers/second_order_controller.py);
       - integrates it into a velocity command, limited to
         max_horizontal_speed;
       - adds the safety push if a neighbour is too close.
     With --controller goal_only_second_order the drones ignore each other.
  3. Land (fly_safely). Save CSV + JSON with metrics (also after an abort).

Expected: the same triangle as the first-order target swarm (same
surprise, so the same resting point), reached with smoother commands.

Run from the repo root:
    python -m src.experiments.second_order_swarm_experiment --mode sim --controller second_order
    python -m src.experiments.second_order_swarm_experiment --mode sim --controller goal_only_second_order
    Fitts sweep (target this far in +x from the start centre, metres):
    python -m src.experiments.second_order_swarm_experiment --mode sim --controller second_order --target-distance 0.8
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
from src.utils.motion_metrics import command_jerk, fitts_id, peak_speed
from src.utils.observation import Observer
from src.utils.run_log import save_run
from src.utils.safety import watch
from src.controllers.second_order_controller import (RateFilter, clamp, limit_speed,
                                                      safety_push, swarm_accelerations)
from src.experiments.target_swarm_experiment import metrics as first_order_metrics
from src.experiments.target_swarm_experiment import pair_distances


DEFAULT_CONFIG = "config/experiments/second_order_swarm.yaml"
N_DRONES = 3
PAIRS = list(itertools.combinations(range(N_DRONES), 2))


def sweep_target(config, distance):
    """Target `distance` metres in +x from the centre of the start positions."""
    cx = sum(p[0] for p in config["start_positions"]) / N_DRONES
    cy = sum(p[1] for p in config["start_positions"]) / N_DRONES
    return [cx + distance, cy]


def run_mission(drones, config, states, variant, observer, rows, placed=False):
    """
    The control loop (see the top of this file).

    Input:  drones   — 3 connected, hovering drones (from fly_safely)
            config   — default.yaml + second_order_swarm.yaml
            states   — live TRUE telemetry (states[0] = CF1, ...)
            variant  — "second_order" or "goal_only_second_order"
            observer — adds the optional --noise to what the controller sees
            rows     — empty list; one dictionary per step is appended
            placed   — True: drones were placed on the start positions by
                       hand, so skip the flight there
    Output: none.

    Called from: fly_safely(), via main().
    """
    if not placed:
        fly_to_positions(drones, states, config, config["start_positions"])
    watch(states, config, config["settle_time"])   # let the hover settle
    target = config["target"]
    height = config["height"]
    dt = config["control_period"]
    print(f"\nController: {variant}   target: {[round(v, 2) for v in target]}   "
          f"desired spacing: {config['desired_distance']} m")

    # Smoothed velocities (for the goal damper) and pair-distance rates
    # (for the neighbour dampers); velocity commands start at rest.
    vel_filters = [(RateFilter(config["rate_filter"]), RateFilter(config["rate_filter"]))
                   for _ in range(N_DRONES)]
    pair_filters = {p: RateFilter(config["rate_filter"]) for p in PAIRS}
    v_cmd = [(0.0, 0.0)] * N_DRONES

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

            # --- What the drones see
            seen = [observer.see(s) for s in states]
            pos = [(o["x"], o["y"]) for o in seen]
            vel = [(fx.update(o["vx"]), fy.update(o["vy"]))
                   for o, (fx, fy) in zip(seen, vel_filters)]
            pair_rates = {}
            for i, j in PAIRS:
                dx, dy = pos[i][0] - pos[j][0], pos[i][1] - pos[j][1]
                d = max(math.hypot(dx, dy), 1e-6)
                measured = ((seen[i]["vx"] - seen[j]["vx"]) * dx +
                            (seen[i]["vy"] - seen[j]["vy"]) * dy) / d
                pair_rates[(i, j)] = pair_filters[(i, j)].update(measured)
                if config["damp_on"] == "command":
                    pair_rates[(i, j)] = ((v_cmd[i][0] - v_cmd[j][0]) * dx +
                                          (v_cmd[i][1] - v_cmd[j][1]) * dy) / d
            if config["damp_on"] == "command":
                # Lighthouse velocity noise times the dampers made the real
                # commands flip direction every step; the commanded velocity
                # is noise-free and the drone tracks it closely.
                vel = list(v_cmd)

            # --- Second-order step: integrate acceleration, then safety push
            acc = swarm_accelerations(pos, vel, pair_rates, target, config, variant)
            pushes = safety_push(pos, config)
            commands = []
            for n, (c, (px, py, active)) in enumerate(zip(acc, pushes)):
                vx, vy = limit_speed(v_cmd[n][0] + c["ax"] * dt, v_cmd[n][1] + c["ay"] * dt,
                                     config["max_horizontal_speed"])
                if active:
                    vx, vy = limit_speed(vx + px, vy + py, config["max_horizontal_speed"])
                v_cmd[n] = (vx, vy)   # the integrator continues from what was sent
                commands.append({**c, "vx": vx, "vy": vy, "protective": active})

            for scf, o, c in zip(drones, seen, commands):
                vz = clamp(config["altitude_kp"] * (height - o["z"]),
                           -config["max_vertical_speed"], config["max_vertical_speed"])
                scf.cf.commander.send_velocity_world_setpoint(c["vx"], c["vy"], vz, 0.0)

            # --- Log TRUE positions + what each drone decided
            cx = sum(p[0] for p in true_xy) / N_DRONES
            cy = sum(p[1] for p in true_xy) / N_DRONES
            row = {"t": round(t, 3), "variant": variant,
                   "centre_error": math.hypot(cx - target[0], cy - target[1]),
                   "centre_speed": math.hypot(sum(c["vx"] for c in commands) / N_DRONES,
                                              sum(c["vy"] for c in commands) / N_DRONES)}
            for i, (s, c) in enumerate(zip(states, commands)):
                n = i + 1
                row.update({f"x{n}": s["x"], f"y{n}": s["y"], f"z{n}": s["z"],
                            f"vx{n}": c["vx"], f"vy{n}": c["vy"],
                            f"ax{n}": c["ax"], f"ay{n}": c["ay"],
                            f"S_goal{n}": c["S_goal"], f"S_neigh{n}": c["S_neighbours"],
                            f"F{n}": c["F"], f"protect{n}": c["protective"]})
            row.update(dists)
            row["F_total"] = sum(c["S_goal"] + c["S_neighbours"] for c in commands)
            rows.append(row)

            if len(rows) % 10 == 1:   # about once per second
                print(f"t={t:5.1f}s  centre_err={row['centre_error']:.2f} m  "
                      f"speed={row['centre_speed']:.3f}  "
                      f"d12={dists['d12']:.2f} d13={dists['d13']:.2f} d23={dists['d23']:.2f}  "
                      f"F_total={row['F_total']:7.2f}"
                      f"{'  PROTECT' if any(c['protective'] for c in commands) else ''}")

            watch(states, config, max(0.0, dt - (time.time() - step_start)))

    finally:
        for scf in drones:   # stop moving before fly_safely() lands them
            try:
                scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass


def metrics(rows, config):
    """
    Per-run results: all target-swarm metrics, plus
      start_centre_error (m) — distance of the swarm centre from the target
                               at the start (Fitts D)
      fitts_id (bits)        — log2(D / W + 1), W = 2 * arrival_tolerance
      centre_overshoot (m)   — how far the centre passed the target along
                               the start direction (0 = never passed)
      peak_speed (m/s), time_to_peak (s), command_jerk (m/s^2)
    """
    result = first_order_metrics(rows, config)
    target = config["target"]
    first = rows[0]
    cx0 = sum(first[f"x{n}"] for n in (1, 2, 3)) / N_DRONES
    cy0 = sum(first[f"y{n}"] for n in (1, 2, 3)) / N_DRONES
    D = math.hypot(target[0] - cx0, target[1] - cy0)
    ux, uy = (target[0] - cx0) / max(D, 1e-6), (target[1] - cy0) / max(D, 1e-6)

    # Progress past the target along the start direction (>0 = overshot).
    passed = 0.0
    for r in rows:
        cx = sum(r[f"x{n}"] for n in (1, 2, 3)) / N_DRONES
        cy = sum(r[f"y{n}"] for n in (1, 2, 3)) / N_DRONES
        passed = max(passed, (cx - target[0]) * ux + (cy - target[1]) * uy)
    peak, t_peak = peak_speed(rows, N_DRONES)

    result.update({
        "start_centre_error": round(D, 3),
        "fitts_id": round(fitts_id(D, 2 * config["arrival_tolerance"]), 3),
        "centre_overshoot": round(passed, 3),
        "peak_speed": round(peak, 3),
        "time_to_peak": round(t_peak, 2),
        "command_jerk": round(command_jerk(rows, N_DRONES, config["control_period"]), 4),
    })
    return result


def main():
    parser = cli.make_parser(DEFAULT_CONFIG, "Three drones to a target, second-order surprise descent")
    parser.add_argument("--controller", choices=["second_order", "goal_only_second_order"],
                        required=True,
                        help="second_order = target + neighbours (proposed); "
                             "goal_only_second_order = target only (baseline)")
    parser.add_argument("--target-distance", type=float,
                        help="Fitts sweep: put the target this many metres in +x "
                             "from the start centre (overrides `target`)")
    parser.add_argument("--placed", action="store_true",
                        help="real drones: you placed them on the start positions by hand, "
                             "so skip the flight there")
    args = parser.parse_args()
    config, seed = cli.load(args)
    if args.target_distance is not None:
        config["target"] = sweep_target(config, args.target_distance)
    observer = Observer(args.noise, seed)
    if args.placed:
        print("Place the drones by hand at (x, y): " +
              ", ".join(f"CF{i + 1} ({x:+.2f}, {y:+.2f})"
                        for i, (x, y) in enumerate(config["start_positions"])))

    rows = []
    aborted = True   # stays True unless the flight finishes normally
    try:
        fly_safely(config, N_DRONES,
                   lambda drones, cfg, states: run_mission(drones, cfg, states, args.controller,
                                                           observer, rows, args.placed))
        aborted = False
    finally:
        if rows:
            result = metrics(rows, config)
            save_run(rows, config, "second_order_swarm", args.controller,
                     {"target_distance": args.target_distance,
                      "noise_std": args.noise, "seed": seed, "aborted": aborted,
                      "flown_s": rows[-1]["t"], "metrics": result})
            for key, value in result.items():
                print(f"  {key:22s} {value}")
            if aborted:
                print("NOTE: this run was ABORTED — exclude it when comparing.")
        else:
            print("No data recorded.")


if __name__ == "__main__":
    main()
