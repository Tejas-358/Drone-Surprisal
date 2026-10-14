"""
convoy_baseline_experiment.py — E2: the leader–follower convoy WITHOUT
surprise gating (the baseline that E1 is compared against).

What happens in one run:
  1. Connect to 3 drones, pre-flight check, take off (done by fly_safely).
  2. Fly into a line along x: CF1 in front, CF2 and CF3 behind.
  3. Control loop: CF1 flies towards a virtual wall and stops in front of
     it. CF2 follows CF1, CF3 follows CF2, using plain P-control (plus the
     safety repulsion) from src/controllers/convoy_controller.py.
     Every step, safety.check_in_flight() aborts if anything is unsafe.
  4. Land and disconnect (done by fly_safely), save a CSV — also after
     an abort, so the data up to the problem is kept.

The control loop lives in run_convoy() so that E1
(surprise_minimization_experiment.py) can reuse it with use_surprise=True.
That guarantees E1 and E2 fly exactly the same way except for the gating.

Run from the repo root:
    sim  (CrazySim running with 3 drones):
        python -m src.experiments.convoy_baseline_experiment
    real (small arena):
        python -m src.experiments.convoy_baseline_experiment --mode real --config config/experiments/convoy_real.yaml
"""

import warnings

warnings.filterwarnings(
    "ignore",
    message="Using legacy TYPE_VELOCITY_WORLD_LEGACY.*",
    category=DeprecationWarning,
)

import argparse
import csv
import os
import time
from datetime import datetime

from src.utils.connection import load_config
from src.utils.flight import fly_safely
from src.utils.safety import check_in_flight, watch
from src.controllers.convoy_controller import leader_command, follower_command, clamp


DEFAULT_CONFIG = "config/experiments/convoy.yaml"

# Gentle correction that keeps each drone on its lane (y = 0) and at its
# height while it moves along x. Without it, velocity commands alone let
# y and z slowly drift on a real drone.
HOLD_GAIN = 1.0        # m/s of correction per metre of error
HOLD_MAX_SPEED = 0.2   # m/s, limit on that correction


def send_velocity(scf, vx, state, config):
    """
    Send a forward (x) velocity command to one drone, while holding y = 0
    and z = flight height.

    Input:  scf    — one connected drone
            vx     — velocity along x in m/s (world frame), from the controller
            state  — this drone's latest telemetry (uses y, z)
            config — uses height
    Output: none.

    Called from: run_convoy(), every control step for each drone that is moving.
    """
    vy = clamp(HOLD_GAIN * (0.0 - state["y"]), -HOLD_MAX_SPEED, HOLD_MAX_SPEED)
    vz = clamp(HOLD_GAIN * (config["height"] - state["z"]), -HOLD_MAX_SPEED, HOLD_MAX_SPEED)
    scf.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0.0)


def fly_to_start_line(drones, states, config):
    """
    Move the hovering drones to their starting x positions (a line along x).

    Input:  drones — list of 3 connected, hovering drones
            states — live telemetry, for safety checks while flying there
            config — uses start_x, height, formation_time
    Output: none.

    Called from: run_convoy(), before the control loop starts.
    """
    print("\nFlying to start line...")
    for scf, x in zip(drones, config["start_x"]):
        # go_to(x, y, z, yaw, duration): high-level "fly to this point" command
        scf.cf.high_level_commander.go_to(x, 0.0, config["height"], 0.0,
                                          config["formation_time"])
    watch(states, config, config["formation_time"] + 1.0)   # +1 s to settle


def run_convoy(drones, config, states, use_surprise, rows):
    """
    The control loop shared by E1 and E2.

    Input:  drones       — list of 3 connected, hovering drones (from fly_safely)
            config       — default.yaml + convoy settings
            states       — live telemetry from fly_safely (states[0] = CF1)
            use_surprise — False for E2 (baseline), True for E1 (gated)
            rows         — an empty list; one dictionary per control step is
                           appended to it. (Passed in, rather than returned,
                           so the caller still has the data if we abort.)
    Output: none.

    Called from: fly_safely() (as the 'mission'), via main() here or in
    surprise_minimization_experiment.py.
    """
    try:
        fly_to_start_line(drones, states, config)

        print("Starting convoy control loop...")
        leader_stopped = False
        leader_hold_x = None   # where the leader parks once it has stopped
        start_time = time.time()

        while True:
            step_start = time.time()
            t = step_start - start_time
            if t >= config["duration"]:
                break

            # Stop the flight if a drone left the arena, flies away, data is stale, or
            # two drones are too close. min_gap_abort is optional in the config.
            check_in_flight(states, config, config.get("min_gap_abort"))

            # Snapshot the latest telemetry (index 0 = CF1, 1 = CF2, 2 = CF3).
            x = [s["x"] for s in states]
            v = [s["vx"] for s in states]

            # --- Leader (CF1)
            leader_vx, leader_stopped = leader_command(x[0], leader_stopped, config)

            # --- Followers: CF2 follows CF1, CF3 follows CF2
            cf2 = follower_command(x[0], x[1], v[0], v[1], config, use_surprise)
            cf3 = follower_command(x[1], x[2], v[1], v[2], config, use_surprise)

            # --- Send all three commands
            if leader_stopped:
                # "Velocity 0" alone lets the leader drift. Once stopped, park
                # it at a fixed point with a position setpoint instead.
                if leader_hold_x is None:
                    leader_hold_x = x[0]
                drones[0].cf.commander.send_position_setpoint(
                    leader_hold_x, 0.0, config["height"], 0.0)
            else:
                send_velocity(drones[0], leader_vx, states[0], config)
            send_velocity(drones[1], cf2["command"], states[1], config)
            send_velocity(drones[2], cf3["command"], states[2], config)

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

            # Sleep only for what is LEFT of the control period, so the loop
            # keeps a steady 10 Hz even when a step took some time.
            step_time = time.time() - step_start
            if step_time > 0.3:
                print(f"WARNING: control step took {step_time:.2f} s")
            time.sleep(max(0.0, config["control_period"] - step_time))

    finally:
        # Stop horizontal motion before fly_safely() lands the drones.
        for scf in drones:
            try:
                scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass


def save_csv(rows, config, experiment_name):
    """
    Write the recorded rows to data/raw/<experiment>_<mode>_<timestamp>.csv.

    Input:  rows            — list of dictionaries from run_convoy()
            config          — uses log_dir, mode
            experiment_name — e.g. "convoy_baseline"
    Output: the path of the file written.

    Called from: main(), after the drones have landed (or aborted).
    """
    os.makedirs(config["log_dir"], exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = os.path.join(config["log_dir"],
                        f"{experiment_name}_{config['mode']}_{stamp}.csv")

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    return path


def run_experiment(experiment_name, use_surprise):
    """
    Parse the command line, fly one convoy run, and save its CSV.

    Input:  experiment_name — used in the CSV file name
            use_surprise    — False for E2, True for E1
    Output: none.

    Called from: main() here (E2) and surprise_minimization_experiment.py (E1).
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--mode", choices=["sim", "real"], help="override mode in config")
    args = parser.parse_args()

    config = load_config(args.config)
    if args.mode:
        config["mode"] = args.mode

    rows = []
    try:
        fly_safely(config, 3,
                   lambda drones, cfg, states: run_convoy(drones, cfg, states,
                                                          use_surprise, rows))

    finally:
        # Save whatever was recorded, even if the run was aborted.
        if rows:
            print(f"Saved {len(rows)} rows to {save_csv(rows, config, experiment_name)}")
        else:
            print("No data recorded.")


def main():
    # use_surprise=False -> plain P-control (the baseline)
    run_experiment("convoy_baseline", use_surprise=False)


if __name__ == "__main__":
    main()
