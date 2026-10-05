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

from src.utils.connection import load_config, connect_all, disconnect_all
from src.utils import sensors


READ_SECONDS = 5.0   # how long to watch the telemetry


def distance_between(p1, p2):
    """
    Straight-line (3D) distance between two drones.

    Input:  p1, p2 — state dictionaries with keys "x", "y", "z" (metres)
    Output: distance in metres.

    Called from: main(), to print neighbour distances.
    """
    dx = p1["x"] - p2["x"]
    dy = p1["y"] - p2["y"]
    dz = p1["z"] - p2["z"]
    return math.sqrt(dx**2 + dy**2 + dz**2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drones", type=int, default=3, help="how many drones to check")
    args = parser.parse_args()

    config = load_config()
    names = [f"CF{i + 1}" for i in range(args.drones)]

    drones = connect_all(config, args.drones)
    log_configs = []

    try:
        # states[i] is updated in the background with drone i's latest data.
        states, log_configs = sensors.start_all(drones, config["log_period_ms"])

        print(f"\nReading positions for {READ_SECONDS:.0f} s...\n")
        start_time = time.time()

        while time.time() - start_time < READ_SECONDS:
            for name, state in zip(names, states):
                if state:
                    print(f"{name}: x={state['x']:.3f}  y={state['y']:.3f}  z={state['z']:.3f}")
                else:
                    print(f"{name}: waiting for telemetry...")

            # Distance from each drone to the next one in the list.
            for i in range(len(states) - 1):
                if states[i] and states[i + 1]:
                    d = distance_between(states[i], states[i + 1])
                    print(f"{names[i]} <-> {names[i + 1]}: {d:.3f} m")

            print("-" * 40)
            time.sleep(1.0)

        # Final verdict: did every drone send at least one packet?
        missing = [name for name, state in zip(names, states) if not state]
        if missing:
            print(f"FAIL: no telemetry from {', '.join(missing)}")
        else:
            print("PASS: all drones connected and sending telemetry.")

    finally:
        sensors.stop_all(log_configs)
        disconnect_all(drones)


if __name__ == "__main__":
    main()
