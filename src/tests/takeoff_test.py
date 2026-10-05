"""
takeoff_test.py — sanity check: can ONE drone take off, hover and land?

Uses fly_safely() from src/utils/flight.py, so this is also the first
test of the shared takeoff / safe-landing code.

When to use: after test_connection passes, before any experiment.

Run from the repo root (CrazySim running, at least 1 drone):
    python -m src.tests.takeoff_test
Press Ctrl-C while it hovers to check the drone still lands safely.
"""

import time

from src.utils.connection import load_config
from src.utils.flight import fly_safely


HOVER_SECONDS = 3.0   # how long to hover before landing


def hover(drones, config):
    """
    The 'mission' for this test: just hover in place.

    Input:  drones — list of connected drones (from fly_safely)
            config — dictionary from load_config()
    Output: none.

    Called from: fly_safely(), after takeoff. Landing happens
    automatically when this function returns (or crashes).
    """
    print(f"Hovering for {HOVER_SECONDS:.0f} s...")
    time.sleep(HOVER_SECONDS)


def main():
    config = load_config()
    fly_safely(config, 1, hover)   # 1 drone = CF1 only
    print("Done.")


if __name__ == "__main__":
    main()
