"""
convoy_baseline_experiment.py — E2: the leader–follower convoy WITHOUT
surprise gating (the baseline that E1 is compared against).

What happens in one run:
  1. Connect to 3 drones, take off (done by fly_safely).
  2. Fly into a line along x: CF1 in front, CF2 and CF3 behind.
  3. Control loop: CF1 flies towards a virtual wall and stops in front of
     it. CF2 follows CF1, CF3 follows CF2, using plain P-control (plus the
     safety repulsion) from src/controllers/convoy_controller.py.
  4. Land and disconnect (done by fly_safely), save a CSV.

The control loop lives in run_convoy() so that E1
(surprise_minimization_experiment.py) can reuse it with use_surprise=True.
That guarantees E1 and E2 fly exactly the same way except for the gating.

Run from the repo root (CrazySim running with 3 drones):
    python -m src.experiments.convoy_baseline_experiment
"""

import argparse
import csv
import os
import time
from datetime import datetime

from src.utils.connection import load_config
from src.utils.flight import fly_safely
from src.utils import sensors
from src.controllers.convoy_controller import leader_command, follower_command


DEFAULT_CONFIG = "config/experiments/convoy.yaml"


def send_velocity(scf, vx):
    """
    Send a forward (x) velocity command to one drone, keeping its height.

    Input:  scf — one connected drone
            vx  — velocity along x in m/s (world frame); y, z and yaw-rate are 0
    Output: none.

    Called from: run_convoy(), every control step for every drone.
    """
    scf.cf.commander.send_velocity_world_setpoint(vx, 0.0, 0.0, 0.0)


def fly_to_start_line(drones, config):
    """
    Move the hovering drones to their starting x positions (a line along x).

    Input:  drones — list of 3 connected, hovering drones
            config — uses start_x, height, formation_time
    Output: none.

    Called from: run_convoy(), before the control loop starts.
    """
    print("\nFlying to start line...")
    for scf, x in zip(drones, config["start_x"]):
        # go_to(x, y, z, yaw, duration): high-level "fly to this point" command
        scf.cf.high_level_commander.go_to(x, 0.0, config["height"], 0.0,
                                          config["formation_time"])
    time.sleep(config["formation_time"] + 1.0)   # +1 s to settle


def run_convoy(drones, config, use_surprise):
    """
    The control loop shared by E1 and E2.

    Input:  drones       — list of 3 connected, hovering drones (from fly_safely)
            config       — default.yaml + convoy.yaml settings
            use_surprise — False for E2 (baseline), True for E1 (gated)
    Output: list of rows (one per control step), each a dictionary of the
            values to save in the CSV.

    Called from: fly_safely() (as the 'mission'), via main() here or in
    surprise_minimization_experiment.py.
    """
    states, log_configs = sensors.start_all(drones, config["log_period_ms"])
    rows = []

    try:
        sensors.wait_for_data(states)
        fly_to_start_line(drones, config)

        print("Starting convoy control loop...")
        leader_stopped = False
        start_time = time.time()

        while True:
            t = time.time() - start_time
            if t >= config["duration"]:
                break

            # Snapshot the latest telemetry (index 0 = CF1, 1 = CF2, 2 = CF3).
            x = [s["x"] for s in states]
            v = [s["vx"] for s in states]

            # --- Leader (CF1)
            leader_vx, leader_stopped = leader_command(x[0], leader_stopped, config)

            # --- Followers: CF2 follows CF1, CF3 follows CF2
            cf2 = follower_command(x[0], x[1], v[0], v[1], config, use_surprise)
            cf3 = follower_command(x[1], x[2], v[1], v[2], config, use_surprise)

            # --- Send all three commands
            send_velocity(drones[0], leader_vx)
            send_velocity(drones[1], cf2["command"])
            send_velocity(drones[2], cf3["command"])

            # --- Record this step (simple CSV for now; the standard
            #     §7 logger will replace this in a later step)
            rows.append({
                "t": t,
                "x1": x[0], "x2": x[1], "x3": x[2],
                "v1": v[0], "v2": v[1], "v3": v[2],
                "leader_stopped": leader_stopped,
                "gap_12": x[0] - x[1],
                "gap_23": x[1] - x[2],
                "S2": cf2["prediction_error"], "S3": cf3["prediction_error"],
                "U2": cf2["surprise"], "U3": cf3["surprise"],
                "factor2": cf2["speed_factor"], "factor3": cf3["speed_factor"],
                "repel2": cf2["repelling"], "repel3": cf3["repelling"],
                "cmd1": leader_vx, "cmd2": cf2["command"], "cmd3": cf3["command"],
            })

            # Print a status line about once per second (every 10th step at 10 Hz).
            if len(rows) % 10 == 1:
                print(f"t={t:5.1f}s  x=[{x[0]:5.2f} {x[1]:5.2f} {x[2]:5.2f}]  "
                      f"gaps=[{x[0] - x[1]:.2f} {x[1] - x[2]:.2f}]  "
                      f"factor=[{cf2['speed_factor']:.2f} {cf3['speed_factor']:.2f}]")

            time.sleep(config["control_period"])

    finally:
        # Stop horizontal motion before fly_safely() lands the drones.
        for scf in drones:
            try:
                send_velocity(scf, 0.0)
            except Exception:
                pass
        sensors.stop_all(log_configs)

    return rows


def save_csv(rows, config, experiment_name):
    """
    Write the recorded rows to data/raw/<experiment>_<timestamp>.csv.

    Input:  rows            — list of dictionaries from run_convoy()
            config          — uses log_dir
            experiment_name — e.g. "convoy_baseline"
    Output: the path of the file written.

    Called from: main(), after the drones have landed.
    """
    os.makedirs(config["log_dir"], exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = os.path.join(config["log_dir"], f"{experiment_name}_{stamp}.csv")

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--mode", choices=["sim", "real"], help="override mode in config")
    args = parser.parse_args()

    config = load_config(args.config)
    if args.mode:
        config["mode"] = args.mode

    # use_surprise=False -> plain P-control (the baseline)
    rows = fly_safely(config, 3,
                      lambda drones, cfg: run_convoy(drones, cfg, use_surprise=False))

    if rows:
        print(f"Saved {len(rows)} rows to {save_csv(rows, config, 'convoy_baseline')}")
    else:
        print("No data recorded.")


if __name__ == "__main__":
    main()
