"""
surprise_minimization_experiment.py — E1: the leader–follower convoy WITH
surprise gating (the proposed method).

It flies exactly the same scenario as E2 (convoy_baseline_experiment.py)
by reusing E2's run_experiment() / run_convoy(). The only difference is
use_surprise=True, which makes each follower multiply its speed by
exp(-surprise_gain * U), where U is its velocity-prediction surprise.
See src/controllers/convoy_controller.py for the maths.

Run from the repo root:
    sim  (CrazySim running with 3 drones):
        python -m src.experiments.surprise_minimization_experiment
    real (small arena):
        python -m src.experiments.surprise_minimization_experiment --mode real --config config/experiments/convoy_real.yaml
"""

from src.experiments.convoy_baseline_experiment import run_experiment


def main():
    # use_surprise=True -> P-control scaled by the surprise factor (E1)
    run_experiment("surprise_minimization", use_surprise=True)


if __name__ == "__main__":
    main()
