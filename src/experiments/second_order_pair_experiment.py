"""
second_order_pair_experiment.py — two drones correct their separation with
a SECOND-ORDER controller (spring + critically damped absorber) while their
midpoint flies to a goal. EXPERIMENTS.md §7.

What happens in one run:
  1. Connect to 2 drones, take off (fly_safely), fly to the start positions
     of the chosen scenario (or of --start-gap for the Fitts sweep).
  2. For `duration` seconds, 10x per second:
       - measure the distance d and its (filtered) rate of change r;
       - error e = d - desired_distance, surprisal S = 0.5 (e / sigma)^2;
       - spring kp (constant, or growing with S), damper kd = 2 sqrt(kp);
       - wanted gap acceleration a = -kp e - kd r;
       - integrate into a gap-rate command u <- clamp(u + a dt);
       - CF1 moves by -u/2 along the line to CF2, CF2 by +u/2, both plus
         the shared goal velocity.
     Below protective_distance a push-apart overrides u; below
     critical_distance the run aborts.
  3. Land (fly_safely). Save CSV + JSON with metrics (also after an abort).

Run from the repo root:
    python -m src.experiments.second_order_pair_experiment --mode sim --controller second_order --scenario too_far
    python -m src.experiments.second_order_pair_experiment --mode sim --controller second_order_adaptive --scenario too_close
    Fitts sweep (start gap in metres):
    python -m src.experiments.second_order_pair_experiment --mode sim --controller second_order --start-gap 0.9
    extra: --noise 0.03 --seed 1   (mimic Lighthouse jitter in sim)
"""

import warnings

# Hide only the known cflib message about the legacy velocity setpoint.
warnings.filterwarnings(
    "ignore",
    message="Using legacy TYPE_VELOCITY_WORLD_LEGACY.*",
    category=DeprecationWarning,
)

import math
import time

from src.utils import cli
from src.utils.flight import fly_safely, fly_to_positions
from src.utils.motion_metrics import command_jerk, fitts_id, peak_speed
from src.utils.observation import Observer
from src.utils.run_log import save_run
from src.utils.safety import watch
from src.controllers.second_order_controller import (RateFilter, clamp, limit_speed,
                                                      pair_step)


DEFAULT_CONFIG = "config/experiments/second_order_pair.yaml"


def start_positions(config, scenario, start_gap):
    """(x, y) start positions for CF1, CF2, or None to start where placed."""
    if start_gap is not None:
        return [[start_gap / 2, 0.0], [-start_gap / 2, 0.0]]
    if scenario == "as_placed":
        return None
    return config["start_positions"][scenario]


def run_mission(drones, config, states, variant, positions, observer, rows):
    """
    The control loop (see the top of this file).

    Input:  drones    — 2 connected, hovering drones (from fly_safely)
            config    — default.yaml + second_order_pair.yaml
            states    — live TRUE telemetry (states[0] = CF1, states[1] = CF2)
            variant   — "second_order" or "second_order_adaptive"
            positions — start positions to fly to first, or None
            observer  — adds the optional --noise to what the controller sees
            rows      — empty list; one dictionary per step is appended
    Output: none.

    Called from: fly_safely(), via main().
    """
    s1, s2 = states
    if positions is not None:
        fly_to_positions(drones, states, config, positions)
    watch(states, config, config["settle_time"])   # let the hover settle

    goal_x = 0.5 * (s1["x"] + s2["x"]) + config["goal_offset"][0]
    goal_y = 0.5 * (s1["y"] + s2["y"]) + config["goal_offset"][1]
    height = config["height"]
    d0 = math.hypot(s2["x"] - s1["x"], s2["y"] - s1["y"])
    print(f"\nStart separation: {d0:.2f} m   controller: {variant}   "
          f"desired: {config['desired_distance']} m")

    rate_filter = RateFilter(config["rate_filter"])
    u = 0.0                      # gap-rate command; starts at rest
    bad_altitude = [0, 0]        # consecutive out-of-range z readings per drone
    start = time.time()
    try:
        while time.time() - start < config["duration"]:
            step_start = time.time()
            t = step_start - start

            # --- Safety on the TRUE positions
            true_d = math.hypot(s2["x"] - s1["x"], s2["y"] - s1["y"])
            if true_d < config["critical_distance"]:
                raise RuntimeError(f"ABORT: drones only {true_d:.2f} m apart")
            # A single Lighthouse glitch can report a wrong z for one sample,
            # so abort only after `altitude_abort_steps` bad readings in a row.
            for n, (name, s) in enumerate((("CF1", s1), ("CF2", s2))):
                if config["min_altitude"] <= s["z"] <= config["max_altitude"]:
                    bad_altitude[n] = 0
                else:
                    bad_altitude[n] += 1
                    print(f"WARNING: {name} at z={s['z']:.2f} m ({bad_altitude[n]} in a row)")
                    if bad_altitude[n] >= config["altitude_abort_steps"]:
                        raise RuntimeError(f"ABORT: {name} at z={s['z']:.2f} m")

            # --- What the controller sees
            o1, o2 = observer.see(s1), observer.see(s2)
            dx, dy = o2["x"] - o1["x"], o2["y"] - o1["y"]
            d = max(math.hypot(dx, dy), 1e-6)
            ux, uy = dx / d, dy / d                       # unit vector CF1 -> CF2
            measured_rate = (o2["vx"] - o1["vx"]) * ux + (o2["vy"] - o1["vy"]) * uy
            rate = rate_filter.update(measured_rate)

            # --- Second-order step
            c = pair_step(d, rate, u, variant, config)
            u = c["u"]

            # --- Shared goal velocity for the midpoint
            mid_x = 0.5 * (o1["x"] + o2["x"])
            mid_y = 0.5 * (o1["y"] + o2["y"])
            goal_err = math.hypot(goal_x - mid_x, goal_y - mid_y)
            if goal_err > config["goal_tolerance"]:
                gvx, gvy = limit_speed(config["goal_kp"] * (goal_x - mid_x),
                                       config["goal_kp"] * (goal_y - mid_y),
                                       config["max_goal_speed"])
            else:
                gvx, gvy = 0.0, 0.0
            if c["protective"]:
                gvx, gvy = 0.25 * gvx, 0.25 * gvy   # separation first

            # --- Each drone: goal velocity + half the gap rate.
            # u > 0 opens the gap: CF1 moves -u/2 along (CF1->CF2), CF2 +u/2.
            vx1, vy1 = limit_speed(gvx - 0.5 * u * ux, gvy - 0.5 * u * uy,
                                   config["max_horizontal_speed"])
            vx2, vy2 = limit_speed(gvx + 0.5 * u * ux, gvy + 0.5 * u * uy,
                                   config["max_horizontal_speed"])

            vz1 = clamp(config["altitude_kp"] * (height - o1["z"]),
                        -config["max_vertical_speed"], config["max_vertical_speed"])
            vz2 = clamp(config["altitude_kp"] * (height - o2["z"]),
                        -config["max_vertical_speed"], config["max_vertical_speed"])

            drones[0].cf.commander.send_velocity_world_setpoint(vx1, vy1, vz1, 0.0)
            drones[1].cf.commander.send_velocity_world_setpoint(vx2, vy2, vz2, 0.0)

            rows.append({
                "t": round(t, 3), "controller": variant,
                "x1": s1["x"], "y1": s1["y"], "z1": s1["z"],
                "x2": s2["x"], "y2": s2["y"], "z2": s2["z"],
                # distance/error: TRUE values; the rest: what the controller used.
                "distance": true_d, "error": true_d - config["desired_distance"],
                "distance_seen": d, "surprisal": c["S"], "kp": c["kp"], "kd": c["kd"],
                "gap_rate_measured": measured_rate, "gap_rate_filtered": rate,
                "accel": c["accel"], "u": u, "protective": c["protective"],
                "goal_error": goal_err,
                "vx1": vx1, "vy1": vy1, "vx2": vx2, "vy2": vy2,
            })

            if len(rows) % 5 == 1:   # print twice per second
                print(f"t={t:5.1f}s  d={true_d:.3f} m  e={c['e']:+.3f}  S={c['S']:6.2f}  "
                      f"kp={c['kp']:.2f} kd={c['kd']:.2f}  u={u:+.3f}"
                      f"{'  PROTECT' if c['protective'] else ''}")

            watch(states, config, max(0.0, config["control_period"] - (time.time() - step_start)))

    finally:
        for scf in drones:   # stop moving before fly_safely() lands them
            try:
                scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass


def metrics(rows, config):
    """
    Per-run results. Same as the first-order two-drone experiment, plus:
      start_error (m)    — |distance - desired| at the start (Fitts D)
      fitts_id (bits)    — log2(D / W + 1), W = 2 * distance_tolerance
      peak_speed (m/s), time_to_peak (s) — fastest commanded drone speed
      command_jerk (m/s^2) — RMS change of the velocity command (smoothness)
    """
    d = [r["distance"] for r in rows]
    e = [r["error"] for r in rows]
    dt = config["control_period"]
    target = config["desired_distance"]
    tol = config["distance_tolerance"]

    settling = ""
    for i in range(len(rows)):
        if all(abs(x) <= tol for x in e[i:]):
            settling = round(rows[i]["t"], 2)
            break
    peak, t_peak = peak_speed(rows, 2)

    return {
        "start_distance": round(d[0], 3),
        "start_error": round(abs(e[0]), 3),
        "fitts_id": round(fitts_id(e[0], 2 * tol), 3),
        "final_distance": round(d[-1], 3),
        "final_error": round(e[-1], 3),
        "min_distance": round(min(d), 3),
        "overshoot": round(max(0.0, target - min(d)) if d[0] > target
                           else max(0.0, max(d) - target), 3),
        "settling_time": settling,
        "rms_error": round(math.sqrt(sum(x * x for x in e) / len(e)), 4),
        "peak_speed": round(peak, 3),
        "time_to_peak": round(t_peak, 2),
        "command_jerk": round(command_jerk(rows, 2, dt), 4),
        "control_effort": round(sum((math.hypot(r["vx1"], r["vy1"]) +
                                     math.hypot(r["vx2"], r["vy2"])) * dt for r in rows), 3),
        "protective_fraction": round(sum(r["protective"] for r in rows) / len(rows), 3),
    }


def main():
    parser = cli.make_parser(DEFAULT_CONFIG, "Two-drone second-order distance experiment")
    parser.add_argument("--controller", choices=["second_order", "second_order_adaptive"],
                        required=True,
                        help="second_order = constant spring; "
                             "second_order_adaptive = spring stiffens with surprise")
    parser.add_argument("--scenario", choices=["as_placed", "too_far", "too_close"],
                        default="as_placed", help="start positions from the config")
    parser.add_argument("--start-gap", type=float,
                        help="Fitts sweep: start this many metres apart along x "
                             "(overrides --scenario)")
    parser.add_argument("--placed", action="store_true",
                        help="real drones: you placed them on the start positions by hand, "
                             "so skip the flight there (the transfer made real drones lose "
                             "Lighthouse tracking)")
    args = parser.parse_args()
    config, seed = cli.load(args)
    observer = Observer(args.noise, seed)

    scenario = f"gap_{args.start_gap:.2f}" if args.start_gap is not None else args.scenario
    positions = start_positions(config, args.scenario, args.start_gap)
    if args.placed and positions is not None:
        print("Place the drones by hand at (x, y): " +
              ", ".join(f"CF{i + 1} ({x:+.2f}, {y:+.2f})" for i, (x, y) in enumerate(positions)))
        positions = None

    rows = []
    aborted = True   # stays True unless the flight finishes normally
    try:
        fly_safely(config, 2,
                   lambda drones, cfg, states: run_mission(drones, cfg, states, args.controller,
                                                           positions, observer, rows))
        aborted = False
    finally:
        if rows:
            result = metrics(rows, config)
            save_run(rows, config, "second_order_pair", args.controller,
                     {"scenario": scenario, "start_gap": args.start_gap,
                      "noise_std": args.noise, "seed": seed, "aborted": aborted,
                      "flown_s": rows[-1]["t"], "metrics": result})
            for key, value in result.items():
                print(f"  {key:20s} {value}")
            if aborted:
                print("NOTE: this run was ABORTED — exclude it when comparing.")
        else:
            print("No data recorded.")


if __name__ == "__main__":
    main()
