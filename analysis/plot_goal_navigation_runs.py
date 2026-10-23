"""
plot_goal_navigation_runs.py — compare all SUCCESSFUL runs of the two-drone
surprisal experiment in one figure.

Reads data/raw/surprisal_goal_navigation_summary.csv, keeps only runs that
finished (aborted = False) in the chosen mode, loads each run's CSV and draws:
  top row    — distance between the drones over time, with the 0.60 m target
  bottom row — surprisal over time
  left column  — "too far" runs (started farther apart than the target)
  right column — "too close" runs (started closer than the target)
Colour = controller (blue = fixed, orange = adaptive).
It also prints the average metrics per controller and scenario.

Analysis never flies a drone; it only reads data/raw/.

Run from the repo root:
    python -m analysis.plot_goal_navigation_runs              # real runs
    python -m analysis.plot_goal_navigation_runs --mode sim   # simulator runs
    python -m analysis.plot_goal_navigation_runs --show      # also open a window
"""

import argparse
import csv
import os

import matplotlib
import matplotlib.pyplot as plt
import yaml


SUMMARY = "data/raw/surprisal_goal_navigation_summary.csv"
CONFIG = "config/experiments/surprisal_goal_navigation.yaml"
OUTPUT = "analysis/figures/goal_navigation_runs_{mode}.png"

# One fixed colour per controller (never by run order).
COLORS = {"fixed": "#2a78d6", "adaptive": "#eb6834"}
METRICS = ["settling_time", "rms_error", "overshoot", "min_distance", "control_effort"]


def scenario_of(row, target):
    """
    "too far" / "too close" for one summary row. Sim runs record the
    scenario they flew; hand-placed real runs ("as_placed", or older rows
    without the column) are classified by their start distance.
    """
    scenario = row.get("scenario", "as_placed")
    if scenario == "too_far":
        return "too far"
    if scenario == "too_close":
        return "too close"
    return "too far" if float(row["start_distance"]) > target else "too close"


def load_runs(mode, target):
    """
    Return the successful runs as a list of dictionaries:
    {"summary": <row from the summary CSV>, "scenario": "too far"/"too close",
     "t": [...], "distance": [...], "surprisal": [...]}
    """
    runs = []
    with open(SUMMARY) as f:
        for row in csv.DictReader(f):
            if row["aborted"] != "False" or row["mode"] != mode:
                continue   # skip aborted runs and runs from the other mode
            path = os.path.join("data/raw", row["run_file"])
            if not os.path.exists(path):
                print(f"Missing run file, skipped: {path}")
                continue
            with open(path) as rf:
                data = list(csv.DictReader(rf))
            runs.append({
                "summary": row,
                "scenario": scenario_of(row, target),
                "t": [float(r["t"]) for r in data],
                "distance": [float(r["distance"]) for r in data],
                "surprisal": [float(r["surprisal"]) for r in data],
            })
    return runs


def print_table(runs):
    """Print the mean of each metric per scenario and controller."""
    print(f"\n{'scenario':10s} {'controller':10s} {'runs':>4s} " +
          " ".join(f"{m:>15s}" for m in METRICS))
    for scenario in ("too far", "too close"):
        for controller in ("fixed", "adaptive"):
            group = [r["summary"] for r in runs
                     if r["scenario"] == scenario and r["summary"]["controller"] == controller]
            if not group:
                print(f"{scenario:10s} {controller:10s} {0:4d}   (no successful runs yet)")
                continue
            means = []
            for m in METRICS:
                values = [float(g[m]) for g in group if g[m] != ""]
                means.append(f"{sum(values) / len(values):15.3f}" if values else f"{'never':>15s}")
            print(f"{scenario:10s} {controller:10s} {len(group):4d} " + " ".join(means))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="real", choices=["sim", "real"])
    parser.add_argument("--show", action="store_true", help="open a window as well")
    args = parser.parse_args()
    if not args.show:
        matplotlib.use("Agg")   # just save the file, no window

    with open(CONFIG) as f:
        exp = yaml.safe_load(f)
    target = exp["desired_distance"]
    tol = exp["distance_tolerance"]

    runs = load_runs(args.mode, target)
    print(f"{len(runs)} successful {args.mode} run(s) found.")
    if not runs:
        return

    fig, axes = plt.subplots(2, 2, figsize=(11, 6.5), sharex=True, sharey="row")
    for col, scenario in enumerate(("too far", "too close")):
        ax_d, ax_s = axes[0][col], axes[1][col]

        # Target and tolerance band, drawn quietly behind the data.
        ax_d.axhspan(target - tol, target + tol, color="#000000", alpha=0.06, lw=0)
        ax_d.axhline(target, color="#666666", lw=1, ls="--")
        ax_d.text(0.3, target + tol + 0.005, f"target {target} m ± {tol}",
                  color="#666666", fontsize=8, va="bottom")

        for r in runs:
            if r["scenario"] != scenario:
                continue
            c = COLORS[r["summary"]["controller"]]
            ax_d.plot(r["t"], r["distance"], color=c, lw=2, alpha=0.85)
            ax_s.plot(r["t"], r["surprisal"], color=c, lw=2, alpha=0.85)

        n = sum(r["scenario"] == scenario for r in runs)
        ax_d.set_title(f"{scenario.capitalize()} start  ({n} run{'s' if n != 1 else ''})", fontsize=11)
        ax_s.set_xlabel("Time (s)")
        for ax in (ax_d, ax_s):
            ax.grid(True, color="#e6e6e6", lw=0.8)
            ax.spines[["top", "right"]].set_visible(False)

    axes[0][0].set_ylabel("Distance between drones (m)")
    axes[1][0].set_ylabel("Surprisal S")

    # Legend for the controllers (always shown: identity is never colour alone).
    handles = [plt.Line2D([], [], color=COLORS[c], lw=2, label=c) for c in ("fixed", "adaptive")]
    fig.suptitle(f"Two-drone surprisal experiment ({args.mode}) — successful runs only", y=0.985, fontsize=12)
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.945),
               ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.90))

    output = OUTPUT.format(mode=args.mode)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"Figure saved to {output}")

    print_table(runs)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
