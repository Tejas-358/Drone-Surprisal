"""
formation_distance_experiment.py — E3: three drones in a line correct
their spacing by gradient descent on a formation potential.

What happens in one run:
  1. Connect to 3 drones, pre-flight check, take off (done by fly_safely).
  2. Fly to a deliberately WRONG start line (gaps 0.3 m and 0.8 m), each
     drone at its own height on the way.
  3. Control loop: every control_period, compute one gradient step with
     src/controllers/formation_controller.py and send each drone to its
     new target x with go_to.
  4. Land (fly_safely), save CSV + JSON (also after an abort) with metrics.

Hypothesis H3: the gaps converge to desired_distance (U goes to ~0).

Run from the repo root:
    sim:   python -m src.experiments.formation_distance_experiment --mode sim
    extra: --noise 0.03 --seed 1   (mimic Lighthouse jitter in sim)
"""

import warnings

warnings.filterwarnings(
    "ignore",
    message="Using legacy TYPE_VELOCITY_WORLD_LEGACY.*",
    category=DeprecationWarning,
)

import time

from src.utils import cli
from src.utils.flight import fly_safely
from src.utils.observation import Observer
from src.utils.run_log import save_run
from src.utils.safety import check_in_flight
from src.controllers.formation_controller import formation_step
from src.experiments.convoy_baseline_experiment import fly_to_start_line


DEFAULT_CONFIG = "config/experiments/formation_distance.yaml"


def run_formation(drones, config, states, rows, observer):
    """
    The control loop for E3.

    Input:  drones   — list of 3 connected, hovering drones (from fly_safely)
            config   — default.yaml + formation_distance.yaml settings
            states   — live telemetry from fly_safely (states[0] = CF1)
            rows     — empty list; one dictionary per step is appended
            observer — adds the optional --noise to what the controller sees
    Output: none.

    Called from: fly_safely() (as the 'mission'), via main().
    """
    fly_to_start_line(drones, states, config)

    print("Starting formation controller...")
    start_time = time.time()

    while True:
        t = time.time() - start_time
        if t >= config["duration"]:
            break

        # Safety uses the TRUE state (arena, fly-away, stale data).
        check_in_flight(states, config)

        # What the controller sees (with --noise, a noisy copy).
        x = [observer.see(s)["x"] for s in states]

        step = formation_step(x[0], x[1], x[2], config)
        targets = [step["target1"], step["target2"], step["target3"]]

        # go_to(x, y, z, yaw, duration): fly to the new target over one
        # control period, so the drone arrives just as the next step starts.
        for scf, target_x in zip(drones, targets):
            scf.cf.high_level_commander.go_to(target_x, 0.0, config["height"],
                                              0.0, config["control_period"])

        true_x = [s["x"] for s in states]
        rows.append({
            "t": round(t, 3),
            "x1": true_x[0], "x2": true_x[1], "x3": true_x[2],
            "gap_12": true_x[1] - true_x[0],
            "gap_23": true_x[2] - true_x[1],
            "S12": step["S12"], "S23": step["S23"], "U": step["U"],
            "step1": step["step1"], "step2": step["step2"], "step3": step["step3"],
            "target1": targets[0], "target2": targets[1], "target3": targets[2],
        })

        print(f"t={t:5.1f}s  gaps=[{rows[-1]['gap_12']:.3f} {rows[-1]['gap_23']:.3f}]  "
              f"U={step['U']:.5f}")

        time.sleep(config["control_period"])


def metrics(rows, config):
    """
    Per-run results for H3 (distances in m, times in s):
      convergence_time — first time after which both gaps stay within
                         gap_tolerance of desired_distance (empty if never)
      final_gap_error  — mean |gap - desired_distance| at the end
      final_U          — formation potential at the end (0 = perfect)
      min_gap          — smallest gap seen (safety)
    """
    target = config["desired_distance"]
    tol = config.get("gap_tolerance", 0.05)
    gaps = [(r["gap_12"], r["gap_23"]) for r in rows]

    convergence = ""
    for i in range(len(rows)):
        if all(abs(a - target) <= tol and abs(b - target) <= tol for a, b in gaps[i:]):
            convergence = round(rows[i]["t"], 2)
            break

    return {
        "convergence_time": convergence,
        "final_gap_error": round(sum(abs(g - target) for g in gaps[-1]) / 2, 3),
        "final_U": round(rows[-1]["U"], 5),
        "min_gap": round(min(min(g) for g in gaps), 3),
    }


def main():
    parser = cli.make_parser(DEFAULT_CONFIG, "E3: formation-distance gradient descent")
    args = parser.parse_args()
    config, seed = cli.load(args)
    observer = Observer(args.noise, seed)

    rows = []
    aborted = True   # stays True unless the flight finishes normally
    try:
        fly_safely(config, 3,
                   lambda drones, cfg, states: run_formation(drones, cfg, states, rows, observer))
        aborted = False
    finally:
        if rows:
            result = metrics(rows, config)
            save_run(rows, config, "formation", "gradient",
                     {"noise_std": args.noise, "seed": seed, "aborted": aborted,
                      "flown_s": rows[-1]["t"], "metrics": result})
            for key, value in result.items():
                print(f"  {key:18s} {value}")
            if aborted:
                print("NOTE: this run was ABORTED — exclude it when comparing.")
        else:
            print("No data recorded.")


if __name__ == "__main__":
    main()
