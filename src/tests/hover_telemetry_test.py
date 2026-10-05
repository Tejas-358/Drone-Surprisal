"""
hover_telemetry_test.py — sanity check: does ONE drone hover steadily, and is its
position telemetry arriving while it flies?

CF1 takes off, then prints its estimated x / y / z five times a second
for 6 seconds, then lands. You should see z settle near the configured
height and x / y stay roughly still.

When to use: after takeoff_test passes, to check that position data
(which every experiment relies on) is sensible during flight.

Run from the repo root (CrazySim running, at least 1 drone):
    python -m src.tests.hover_telemetry_test
"""

import time

from src.utils.connection import load_config
from src.utils.flight import fly_safely
from src.utils import sensors


WATCH_SECONDS = 6.0    # how long to print positions while hovering
PRINT_PERIOD = 0.2     # seconds between printed lines


def watch_position(drones, config):
    """
    The 'mission' for this test: hover and print CF1's position.

    Input:  drones — list with 1 connected drone (from fly_safely)
            config — dictionary from load_config() (uses log_period_ms, height)
    Output: none.

    Called from: fly_safely(), after takeoff. Landing happens
    automatically when this function returns (or crashes).
    """
    # states[0] is updated in the background with CF1's latest data.
    states, log_configs = sensors.start_all(drones, config["log_period_ms"])
    state = states[0]

    try:
        sensors.wait_for_data(states)   # stop with an error if nothing arrives
        start_time = time.time()

        while time.time() - start_time < WATCH_SECONDS:
            t = time.time() - start_time
            print(
                f"t={t:4.1f}s  x={state['x']:.3f}  "
                f"y={state['y']:.3f}  z={state['z']:.3f}"
            )
            time.sleep(PRINT_PERIOD)

        # Simple verdict: is the drone near the height we asked for?
        if abs(state["z"] - config["height"]) < 0.1:
            print(f"PASS: hovering at z={state['z']:.3f} m "
                  f"(target {config['height']} m).")
        else:
            print(f"CHECK: z={state['z']:.3f} m is more than 0.1 m "
                  f"from the target {config['height']} m.")

    finally:
        sensors.stop_all(log_configs)


def main():
    config = load_config()
    fly_safely(config, 1, watch_position)   # 1 drone = CF1 only
    print("Done.")


if __name__ == "__main__":
    main()
