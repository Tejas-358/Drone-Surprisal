"""
test_connection.py — sanity check: can we talk to the drones? (NO flying)

Connects to the drones, streams their position for 5 seconds, prints
the positions and the distance between neighbours, then disconnects.
The motors are never started.

When to use: first thing in every session, before any flight, to check
that CrazySim (or the radio) is up and telemetry is arriving.

Run from the repo root:
    python -m src.tests.test_connection              # 3 drones (default)
    python -m src.tests.test_connection --drones 1   # only CF1
"""

import argparse
import math
import time

from cflib.crazyflie.log import LogConfig

from src.utils.connection import load_config, connect_all, disconnect_all


READ_SECONDS = 5.0   # how long to watch the telemetry


def distance_between(p1, p2):
    """
    Straight-line (3D) distance between two positions.

    Input:  p1, p2 — dictionaries with keys "x", "y", "z" (metres)
    Output: distance in metres.

    Called from: main(), to print neighbour distances.
    """
    dx = p1["x"] - p2["x"]
    dy = p1["y"] - p2["y"]
    dz = p1["z"] - p2["z"]
    return math.sqrt(dx**2 + dy**2 + dz**2)


def start_position_logging(scf, name, positions, period_ms):
    """
    Ask one drone to stream its estimated position to us.

    Input:  scf       — one connected drone (from connect_all)
            name      — label to store the data under, e.g. "CF1"
            positions — shared dictionary; the latest position is written
                        into positions[name] every time data arrives
            period_ms — how often the drone sends data
    Output: the LogConfig object (so it can be stopped later).

    Called from: main(), once per drone.
    """
    log_config = LogConfig(name=f"Position_{name}", period_in_ms=period_ms)
    log_config.add_variable("stateEstimate.x", "float")
    log_config.add_variable("stateEstimate.y", "float")
    log_config.add_variable("stateEstimate.z", "float")

    # This inner function is called by CFLib (in the background) every
    # time a new packet arrives. It just saves the latest values.
    def callback(timestamp, data, logconf):
        positions[name] = {
            "x": data["stateEstimate.x"],
            "y": data["stateEstimate.y"],
            "z": data["stateEstimate.z"],
        }

    log_config.data_received_cb.add_callback(callback)
    scf.cf.log.add_config(log_config)
    log_config.start()

    return log_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drones", type=int, default=3, help="how many drones to check")
    args = parser.parse_args()

    config = load_config()
    names = [f"CF{i + 1}" for i in range(args.drones)]

    drones = connect_all(config, args.drones)
    positions = {}       # filled in by the logging callbacks
    log_configs = []

    try:
        for name, scf in zip(names, drones):
            log_configs.append(
                start_position_logging(scf, name, positions, config["log_period_ms"])
            )

        print(f"\nReading positions for {READ_SECONDS:.0f} s...\n")
        start_time = time.time()

        while time.time() - start_time < READ_SECONDS:
            for name in names:
                if name in positions:
                    p = positions[name]
                    print(f"{name}: x={p['x']:.3f}  y={p['y']:.3f}  z={p['z']:.3f}")
                else:
                    print(f"{name}: waiting for telemetry...")

            # Distance from each drone to the next one in the list.
            for a, b in zip(names, names[1:]):
                if a in positions and b in positions:
                    d = distance_between(positions[a], positions[b])
                    print(f"{a} <-> {b}: {d:.3f} m")

            print("-" * 40)
            time.sleep(1.0)

        # Final verdict: did every drone send at least one position?
        missing = [name for name in names if name not in positions]
        if missing:
            print(f"FAIL: no telemetry from {', '.join(missing)}")
        else:
            print("PASS: all drones connected and sending telemetry.")

    finally:
        for log_config in log_configs:
            try:
                log_config.stop()
            except Exception:
                pass
        disconnect_all(drones)


if __name__ == "__main__":
    main()
