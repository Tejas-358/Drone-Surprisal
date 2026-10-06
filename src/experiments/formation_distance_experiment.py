"""
formation_distance_experiment.py — E3: three drones in a line correct
their spacing by gradient descent on a formation potential.

What happens in one run:
  1. Connect to 3 drones, take off (done by fly_safely).
  2. Fly to a deliberately WRONG start line (gaps 0.3 m and 0.8 m).
  3. Control loop: every control_period, compute one gradient step with
     src/controllers/formation_controller.py and send each drone to its
     new target x with go_to.
  4. Land and disconnect (done by fly_safely), save a CSV.

Hypothesis H3: the gaps converge to desired_distance (U goes to ~0).

Run from the repo root (CrazySim running with 3 drones):
    python -m src.experiments.formation_distance_experiment
"""

import argparse
import time

from src.utils.connection import load_config
from src.utils.flight import fly_safely
from src.utils import sensors
from src.controllers.formation_controller import formation_step
# Reuse the start-line and CSV helpers from the convoy experiment
# (save_csv will be replaced by the standard logger in a later step).
from src.experiments.convoy_baseline_experiment import fly_to_start_line, save_csv


DEFAULT_CONFIG = "config/experiments/formation_distance.yaml"


def run_formation(drones, config):
    """
    The control loop for E3.

    Input:  drones — list of 3 connected, hovering drones (from fly_safely)
            config — default.yaml + formation_distance.yaml settings
    Output: list of rows (one per control step), each a dictionary of the
            values to save in the CSV.

    Called from: fly_safely() (as the 'mission'), via main().
    """
    states, log_configs = sensors.start_all(drones, config["log_period_ms"])
    rows = []

    try:
        sensors.wait_for_data(states)
        fly_to_start_line(drones, config)

        print("Starting formation controller...")
        start_time = time.time()

        while True:
            t = time.time() - start_time
            if t >= config["duration"]:
                break

            # Latest x of CF1, CF2, CF3.
            x = [s["x"] for s in states]

            step = formation_step(x[0], x[1], x[2], config)
            targets = [step["target1"], step["target2"], step["target3"]]

            # go_to(x, y, z, yaw, duration): fly to the new target over one
            # control period, so the drone arrives just as the next step starts.
            for scf, target_x in zip(drones, targets):
                scf.cf.high_level_commander.go_to(target_x, 0.0, config["height"],
                                                  0.0, config["control_period"])

            rows.append({
                "t": t,
                "x1": x[0], "x2": x[1], "x3": x[2],
                "gap_12": x[1] - x[0],
                "gap_23": x[2] - x[1],
                "S12": step["S12"], "S23": step["S23"], "U": step["U"],
                "step1": step["step1"], "step2": step["step2"], "step3": step["step3"],
                "target1": targets[0], "target2": targets[1], "target3": targets[2],
            })

            print(f"t={t:5.1f}s  gaps=[{x[1] - x[0]:.3f} {x[2] - x[1]:.3f}]  "
                  f"U={step['U']:.5f}")

            time.sleep(config["control_period"])

    finally:
        sensors.stop_all(log_configs)

    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--mode", choices=["sim", "real"], help="override mode in config")
    args = parser.parse_args()

    config = load_config(args.config)
    if args.mode:
        config["mode"] = args.mode

    rows = fly_safely(config, 3, run_formation)

    if rows:
        path = save_csv(rows, config, "formation_distance")
        print(f"Saved {len(rows)} rows to {path}")
        print(f"Final gaps: {rows[-1]['gap_12']:.3f} m and {rows[-1]['gap_23']:.3f} m "
              f"(target {config['desired_distance']} m)")
    else:
        print("No data recorded.")


if __name__ == "__main__":
    main()
