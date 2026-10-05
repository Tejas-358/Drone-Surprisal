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

from cflib.crazyflie.log import LogConfig

from src.utils.connection import load_config
from src.utils.flight import fly_safely


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
    scf = drones[0]
    position = {}   # filled in by the callback below

    # Ask the drone to stream its estimated position.
    # (This will move to src/utils/sensors.py in a later step.)
    log_config = LogConfig(name="Position", period_in_ms=config["log_period_ms"])
    log_config.add_variable("stateEstimate.x", "float")
    log_config.add_variable("stateEstimate.y", "float")
    log_config.add_variable("stateEstimate.z", "float")

    # Called by CFLib in the background each time new data arrives.
    def callback(timestamp, data, logconf):
        position["x"] = data["stateEstimate.x"]
        position["y"] = data["stateEstimate.y"]
        position["z"] = data["stateEstimate.z"]

    log_config.data_received_cb.add_callback(callback)
    scf.cf.log.add_config(log_config)
    log_config.start()

    try:
        start_time = time.time()

        while time.time() - start_time < WATCH_SECONDS:
            t = time.time() - start_time
            if position:
                print(
                    f"t={t:4.1f}s  x={position['x']:.3f}  "
                    f"y={position['y']:.3f}  z={position['z']:.3f}"
                )
            else:
                print(f"t={t:4.1f}s  waiting for telemetry...")
            time.sleep(PRINT_PERIOD)

        # Simple verdict: is the drone near the height we asked for?
        if position and abs(position["z"] - config["height"]) < 0.1:
            print(f"PASS: hovering at z={position['z']:.3f} m "
                  f"(target {config['height']} m).")
        else:
            print("CHECK: no telemetry, or height is more than 0.1 m off target.")

    finally:
        log_config.stop()


def main():
    config = load_config()
    fly_safely(config, 1, watch_position)   # 1 drone = CF1 only
    print("Done.")


if __name__ == "__main__":
    main()
