"""
three_takeoff_test.py — sanity check: can THREE drones take off, hover and land together?

Same as takeoff_test.py but with CF1, CF2 and CF3. Uses fly_safely()
from src/utils/flight.py, so all three land even on an error or Ctrl-C.

When to use: before any multi-drone experiment (the convoy experiments
E1/E2 and the formation experiment E3 all use 3 drones).

Run from the repo root (CrazySim running with 3 drones):
    python -m src.tests.three_takeoff_test
"""

import time

from src.utils.connection import load_config
from src.utils.flight import fly_safely


HOVER_SECONDS = 5.0   # how long to hover before landing


def hover(drones, config):
    """
    The 'mission' for this test: all drones just hover in place.

    Input:  drones — list of 3 connected drones (from fly_safely)
            config — dictionary from load_config()
    Output: none.

    Called from: fly_safely(), after all three have taken off.
    Landing happens automatically when this function returns.
    """
    print(f"All {len(drones)} drones hovering for {HOVER_SECONDS:.0f} s...")
    time.sleep(HOVER_SECONDS)


def main():
    config = load_config()
    fly_safely(config, 3, hover)   # 3 drones = CF1, CF2, CF3
    print("Done.")


if __name__ == "__main__":
    main()
