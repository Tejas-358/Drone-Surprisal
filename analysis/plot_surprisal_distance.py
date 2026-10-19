"""
plot_surprisal_distance.py — plot one two-drone surprisal run from its CSV.

Shows the separation over time (with the target distance), the surprisal,
and — only if the CSV has them — the energy columns written by older runs.
Analysis never flies a drone; it only reads data/raw/.

Run from the repo root:
    python -m analysis.plot_surprisal_distance data/raw/<file>.csv
    python -m analysis.plot_surprisal_distance data/raw/<file>.csv --target 0.9
"""

import argparse

import matplotlib.pyplot as plt
import pandas as pd


ENERGY_COLUMNS = ["kinetic_energy_estimate_J",
                  "virtual_distance_potential_proxy",
                  "mechanical_energy_proxy"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="path to a run's CSV in data/raw/")
    parser.add_argument("--target", type=float, default=0.90,
                        help="desired distance to draw as a dashed line (m)")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    # Older runs also logged energies; newer ones do not.
    has_energy = all(c in df.columns for c in ENERGY_COLUMNS)
    n_panels = 3 if has_energy else 2

    fig, axes = plt.subplots(n_panels, 1, figsize=(10, 3.3 * n_panels), sharex=True)

    axes[0].plot(df["time_s"], df["distance_m"])
    axes[0].axhline(args.target, linestyle="--")
    axes[0].set_ylabel("Distance (m)")
    axes[0].set_title(args.csv)
    axes[0].grid(True)

    axes[1].plot(df["time_s"], df["surprisal"])
    axes[1].set_ylabel("Surprisal")
    axes[1].grid(True)

    if has_energy:
        axes[2].plot(df["time_s"], df["kinetic_energy_estimate_J"], label="Kinetic")
        axes[2].plot(df["time_s"], df["virtual_distance_potential_proxy"], label="Virtual potential")
        axes[2].plot(df["time_s"], df["mechanical_energy_proxy"], label="Total")
        axes[2].set_ylabel("Energy (J)")
        axes[2].legend()
        axes[2].grid(True)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
