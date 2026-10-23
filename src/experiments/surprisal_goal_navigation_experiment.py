"""
surprisal_goal_navigation_experiment.py — two drones correct their
separation while their midpoint flies to a goal.

What happens in one run:
  1. Connect to 2 drones, pre-flight check (they must sit on their start
     spots from the config), take off — all done by fly_safely().
  2. For `duration` seconds, 10x per second:
       - measure the distance d between the drones;
       - error   e = d - desired_distance;
       - surprisal S = 0.5 * (e / sigma)^2   (how unexpected the error is);
       - gain    kp = base_kp                            (fixed controller)
                 kp = min(base_kp * (1 + k*S), max_kp)   (adaptive controller)
       - each drone moves along the line between them by 0.5 * (kp*e + damping),
         so they close (e > 0) or open (e < 0) the gap;
       - both also move with the same "goal" velocity that brings their
         midpoint to the goal.
     If the drones get closer than protective_distance, a push-apart
     overrides everything else; below critical_distance the run aborts.
  3. Land (fly_safely). Save one CSV row per step, then compute and print
     the result metrics and add them as one row to a summary CSV.

Run from the repo root:
    real (drones placed by hand):
        python -m src.experiments.surprisal_goal_navigation_experiment --mode real --controller fixed
    sim (drones fly to the start positions of a scenario first):
        python -m src.experiments.surprisal_goal_navigation_experiment --mode sim --controller adaptive --scenario too_far
    extra: --scenario too_far|too_close|as_placed   (default as_placed)
           --noise 0.03 --seed 1                    (mimic Lighthouse jitter in sim)
"""

import warnings

# Hide only the known cflib message about the legacy velocity setpoint.
warnings.filterwarnings(
    "ignore",
    message="Using legacy TYPE_VELOCITY_WORLD_LEGACY.*",
    category=DeprecationWarning,
)

import csv
import math
import os
import time

from src.utils import cli
from src.utils.flight import fly_safely, fly_to_positions
from src.utils.observation import Observer
from src.utils.run_log import save_run
from src.utils.safety import watch


DEFAULT_CONFIG = "config/experiments/surprisal_goal_navigation.yaml"
SUMMARY_CSV = "data/raw/surprisal_goal_navigation_summary.csv"


def clamp(value, low, high):
    """Keep value inside [low, high]."""
    return max(low, min(high, value))


def limit_speed(vx, vy, max_speed):
    """
    Shrink a 2-D velocity (vx, vy) so its size is at most max_speed,
    keeping its direction. Called for every command before it is sent.
    """
    speed = math.hypot(vx, vy)
    if speed > max_speed:
        return vx * max_speed / speed, vy * max_speed / speed
    return vx, vy


def gain(surprisal, controller, config):
    """
    The controller gain kp for this step.

    fixed:    always base_kp (the baseline).
    adaptive: grows with surprisal — a bigger, more "surprising" spacing
              error gets a stiffer correction — up to max_kp.

    Called from: run_mission(), once per step.
    """
    if controller == "fixed":
        return config["base_kp"]
    return min(config["base_kp"] * (1.0 + config["surprise_gain"] * surprisal),
               config["max_kp"])


def run_mission(drones, config, states, controller, scenario, observer, rows):
    """
    The control loop (see the top of this file).

    Input:  drones     — 2 connected, hovering drones (from fly_safely)
            config     — default.yaml + this experiment's yaml
            states     — live telemetry (states[0] = CF1, states[1] = CF2)
            controller — "fixed" or "adaptive"
            scenario   — "too_far" / "too_close": fly to that scenario's start
                         positions first; "as_placed": start where they are
            observer   — adds the optional --noise to what the controller sees
            rows       — empty list; one dictionary per step is appended
                         (passed in so the data survives an abort)
    Output: none.

    Called from: fly_safely(), via main().
    """
    s1, s2 = states          # TRUE states (used for safety and logging)
    if scenario != "as_placed":
        fly_to_positions(drones, states, config, config["start_positions"][scenario])
    watch(states, config, 1.0)   # let the hover settle

    # The goal: the drones' midpoint, shifted by goal_offset.
    goal_x = 0.5 * (s1["x"] + s2["x"]) + config["goal_offset"][0]
    goal_y = 0.5 * (s1["y"] + s2["y"]) + config["goal_offset"][1]
    height = config["height"]

    # Placement is not enforced, so warn about the two things that matter.
    dx0, dy0 = s2["x"] - s1["x"], s2["y"] - s1["y"]
    d0 = math.hypot(dx0, dy0)
    print(f"\nStart separation: {d0:.2f} m")
    # Start "too far" (e > 0: they should pull together) or "too close"
    # (e < 0: they should push apart). Both are valid experiments; only a
    # start right AT the desired distance tests nothing.
    if abs(d0 - config["desired_distance"]) < 0.1:
        print("WARNING: drones start (almost) at the desired distance — there is "
              "little error to correct, so this run says little about the controller.")
    elif d0 > config["desired_distance"]:
        print("Scenario: TOO FAR — the drones should move together.")
    else:
        print("Scenario: TOO CLOSE — the drones should move apart.")
    if abs(dy0) > abs(dx0):
        print("WARNING: drones are lined up mostly along y — one may hide a base "
              "station from the other. Place them along x next time.")

    print(f"\nController: {controller}   desired distance: {config['desired_distance']} m")
    start = time.time()

    try:
        while time.time() - start < config["duration"]:
            step_start = time.time()
            t = step_start - start

            # --- Safety on the TRUE positions
            true_d = math.hypot(s2["x"] - s1["x"], s2["y"] - s1["y"])
            if true_d < config["critical_distance"]:
                raise RuntimeError(f"ABORT: drones only {true_d:.2f} m apart")
            for name, s in (("CF1", s1), ("CF2", s2)):
                if not config["min_altitude"] <= s["z"] <= config["max_altitude"]:
                    raise RuntimeError(f"ABORT: {name} at z={s['z']:.2f} m")

            # --- What the controller sees (with --noise, noisy copies)
            o1, o2 = observer.see(s1), observer.see(s2)
            dx = o2["x"] - o1["x"]
            dy = o2["y"] - o1["y"]
            d = math.hypot(dx, dy)           # measured distance (horizontal)
            ux, uy = dx / d, dy / d          # unit vector from CF1 to CF2

            # --- Surprisal and gain
            e = d - config["desired_distance"]            # >0: too far apart
            S = 0.5 * (e / config["sigma_distance"]) ** 2
            kp = gain(S, controller, config)

            # How fast the gap is changing (>0: drifting apart), from the
            # drones' own velocity estimates.
            gap_rate = (o2["vx"] - o1["vx"]) * ux + (o2["vy"] - o1["vy"]) * uy

            # Positive = close the gap, negative = open it. The damping term
            # resists the gap changing quickly, which reduces overshoot.
            correction = kp * e + config["kd_relative"] * gap_rate

            protective = d < config["protective_distance"]
            if protective:
                # Too close: always push apart, at least this hard.
                push = -config["separation_recovery_gain"] * (config["protective_distance"] - d)
                correction = min(correction, push)

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
            if protective:
                gvx, gvy = 0.25 * gvx, 0.25 * gvy   # separation first

            # --- Each drone: goal velocity + half the correction.
            # CF1 moves towards CF2 (+u), CF2 towards CF1 (-u).
            v1x, v1y = limit_speed(gvx + 0.5 * correction * ux, gvy + 0.5 * correction * uy,
                                   config["max_horizontal_speed"])
            v2x, v2y = limit_speed(gvx - 0.5 * correction * ux, gvy - 0.5 * correction * uy,
                                   config["max_horizontal_speed"])

            # Height hold: small vertical speed towards the flight height.
            vz1 = clamp(config["altitude_kp"] * (height - o1["z"]),
                        -config["max_vertical_speed"], config["max_vertical_speed"])
            vz2 = clamp(config["altitude_kp"] * (height - o2["z"]),
                        -config["max_vertical_speed"], config["max_vertical_speed"])

            drones[0].cf.commander.send_velocity_world_setpoint(v1x, v1y, vz1, 0.0)
            drones[1].cf.commander.send_velocity_world_setpoint(v2x, v2y, vz2, 0.0)

            rows.append({
                "t": round(t, 3), "controller": controller,
                "x1": s1["x"], "y1": s1["y"], "z1": s1["z"],
                "x2": s2["x"], "y2": s2["y"], "z2": s2["z"],
                # distance/error: TRUE values (what really happened);
                # distance_seen, surprisal, kp: what the controller used.
                "distance": true_d, "error": true_d - config["desired_distance"],
                "distance_seen": d, "surprisal": S, "kp": kp,
                "gap_rate": gap_rate, "protective": protective,
                "goal_error": goal_err,
                "v1x": v1x, "v1y": v1y, "v2x": v2x, "v2y": v2y,
            })

            if len(rows) % 5 == 1:   # print twice per second
                print(f"t={t:5.1f}s  d={true_d:.3f} m  e={e:+.3f}  S={S:6.2f}  kp={kp:.2f}  "
                      f"goal_err={goal_err:.2f}{'  PROTECT' if protective else ''}")

            # Safety checks (arena, fly-away, stale data) while waiting for
            # the rest of this control period.
            watch(states, config, max(0.0, config["control_period"] - (time.time() - step_start)))

    finally:
        for scf in drones:   # stop moving before fly_safely() lands them
            try:
                scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass


def metrics(rows, config):
    """
    Turn one run's rows into the numbers we compare between controllers.

    Output: dictionary with
      start_distance, final_distance, final_error (m)
      min_distance (m)          — closest the drones came (safety)
      overshoot (m)             — how far the gap went past the target
                                  (below it, since we start too far apart)
      settling_time (s)         — first time after which |error| stays
                                  within distance_tolerance (empty if never)
      rms_error (m)             — average size of the spacing error
      control_effort (m)        — sum of |commanded speed| x dt, both drones
      protective_fraction       — share of steps in push-apart mode
    """
    d = [r["distance"] for r in rows]
    e = [r["error"] for r in rows]
    dt = config["control_period"]
    target = config["desired_distance"]

    settling = ""
    for i in range(len(rows)):
        if all(abs(x) <= config["distance_tolerance"] for x in e[i:]):
            settling = round(rows[i]["t"], 2)
            break

    return {
        "start_distance": round(d[0], 3),
        "final_distance": round(d[-1], 3),
        "final_error": round(e[-1], 3),
        "min_distance": round(min(d), 3),
        "overshoot": round(max(0.0, target - min(d)) if d[0] > target
                           else max(0.0, max(d) - target), 3),
        "settling_time": settling,
        "rms_error": round(math.sqrt(sum(x * x for x in e) / len(e)), 4),
        "control_effort": round(sum((math.hypot(r["v1x"], r["v1y"]) +
                                     math.hypot(r["v2x"], r["v2y"])) * dt for r in rows), 3),
        "protective_fraction": round(sum(r["protective"] for r in rows) / len(rows), 3),
    }


def save(rows, config, controller, scenario, noise_std, seed, aborted):
    """
    Save the step-by-step CSV + JSON (run_log.save_run) and append one row
    to the summary CSV (one row per run, for quick comparisons).

    aborted — True if the run stopped early (safety abort, radio loss,
              Ctrl-C). Its metrics are then NOT comparable with full runs:
              filter these rows out when comparing the controllers.

    Called from: main(), after landing (also after an abort).
    """
    result = metrics(rows, config)
    path = save_run(rows, config, "goal_navigation", controller,
                    {"scenario": scenario, "noise_std": noise_std, "seed": seed,
                     "aborted": aborted, "flown_s": rows[-1]["t"], "metrics": result})

    summary = {"run_file": os.path.basename(path), "controller": controller,
               "mode": config["mode"], "scenario": scenario,
               "noise_std": noise_std, "seed": seed, "aborted": aborted,
               "flown_s": round(rows[-1]["t"], 1), **result}
    new_file = not os.path.exists(SUMMARY_CSV)
    with open(SUMMARY_CSV, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=summary.keys())
        if new_file:
            writer.writeheader()
        writer.writerow(summary)

    print("Results (also added to " + SUMMARY_CSV + "):")
    for key, value in summary.items():
        print(f"  {key:20s} {value}")


def main():
    parser = cli.make_parser(DEFAULT_CONFIG, "Two-drone surprisal distance experiment")
    parser.add_argument("--controller", choices=["fixed", "adaptive"], required=True,
                        help="fixed = baseline, adaptive = surprisal-adaptive gain")
    parser.add_argument("--scenario", choices=["as_placed", "too_far", "too_close"],
                        default="as_placed",
                        help="fly to this scenario's start positions first "
                             "(needed in sim; on real drones you may also place them by hand)")
    args = parser.parse_args()
    config, seed = cli.load(args)
    observer = Observer(args.noise, seed)

    rows = []
    aborted = True   # stays True unless the flight finishes normally
    try:
        fly_safely(config, 2,
                   lambda drones, cfg, states: run_mission(drones, cfg, states, args.controller,
                                                           args.scenario, observer, rows))
        aborted = False
    finally:
        if rows:
            save(rows, config, args.controller, args.scenario, args.noise, seed, aborted)
            if aborted:
                print("NOTE: this run was ABORTED — exclude it when comparing controllers.")
        else:
            print("No data recorded.")


if __name__ == "__main__":
    main()
