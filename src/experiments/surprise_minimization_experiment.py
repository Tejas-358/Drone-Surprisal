"""
surprise_minimization_experiment.py — E1: the leader–follower convoy WITH
surprise gating (the proposed method).

It flies exactly the same scenario as E2 (convoy_baseline_experiment.py)
by reusing E2's run_convoy() loop. The only difference is
use_surprise=True, which makes each follower multiply its speed by
exp(-surprise_gain * U), where U is its velocity-prediction surprise.
See src/controllers/convoy_controller.py for the maths.

Run from the repo root (CrazySim running with 3 drones):
    python -m src.experiments.surprise_minimization_experiment
"""

import argparse

from src.utils.connection import load_config
from src.utils.flight import fly_safely
from src.experiments.convoy_baseline_experiment import (
    DEFAULT_CONFIG, run_convoy, save_csv,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--mode", choices=["sim", "real"], help="override mode in config")
    args = parser.parse_args()

    config = load_config(args.config)
    if args.mode:
        config["mode"] = args.mode

    # use_surprise=True -> P-control scaled by the surprise factor (E1)
    rows = fly_safely(config, 3,
                      lambda drones, cfg: run_convoy(drones, cfg, use_surprise=True))

    if rows:
        path = save_csv(rows, config, "surprise_minimization")
        print(f"Saved {len(rows)} rows to {path}")
    else:
        print("No data recorded.")


if __name__ == "__main__":
    main()
