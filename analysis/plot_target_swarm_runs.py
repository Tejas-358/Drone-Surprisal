"""
plot_target_swarm_runs.py — results of the 3-drone target experiment
(surprise vs goal_only), from all finished runs in data/raw/.

Figure (analysis/figures/target_swarm_runs_<mode>.png):
  top row     — top-down paths of the three drones in the most recent run of
                each controller, with the target (x) and start points (o)
  bottom left — smallest pair distance over time, every run, with the
                desired spacing and the safety-push distance
  bottom right— total surprise over time, every run
Colour in the bottom row = controller (blue = goal_only, orange = surprise).
It also prints mean +/- std of the metrics per controller.

Run from the repo root:
    python -m analysis.plot_target_swarm_runs --mode sim
    python -m analysis.plot_target_swarm_runs --mode sim --show
"""

import argparse
import os

import matplotlib
import matplotlib.pyplot as plt

from analysis.load_runs import load_runs, group_metrics, print_table


VARIANTS = ["goal_only", "surprise"]
VARIANT_COLORS = {"goal_only": "#2a78d6", "surprise": "#eb6834"}
DRONE_COLORS = ["#1baf7a", "#eda100", "#e87ba4"]   # CF1, CF2, CF3 (not the controller colours)
METRICS = ["arrival_time", "min_pair_distance", "final_spacing_error",
           "protective_fraction", "final_total_surprise"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parser.add_argument("--show", action="store_true", help="open a window as well")
    args = parser.parse_args()
    if not args.show:
        matplotlib.use("Agg")

    runs = load_runs("target_swarm", args.mode)
    print(f"{len(runs)} finished {args.mode} run(s) found.")
    if not runs:
        return

    fig, axes = plt.subplots(2, 2, figsize=(11, 8.5))

    # --- Top row: paths of the latest run per controller
    for col, variant in enumerate(VARIANTS):
        ax = axes[0][col]
        mine = [r for r in runs if r["meta"]["variant"] == variant]
        if not mine:
            ax.set_title(f"{variant}: no runs yet")
            ax.axis("off")
            continue
        run = mine[-1]
        rows, cfg = run["rows"], run["meta"]["config"]
        for n in (1, 2, 3):
            xs = [r[f"x{n}"] for r in rows]
            ys = [r[f"y{n}"] for r in rows]
            ax.plot(xs, ys, color=DRONE_COLORS[n - 1], lw=2, label=f"CF{n}")
            ax.plot(xs[0], ys[0], "o", color=DRONE_COLORS[n - 1], ms=8)
            ax.plot(xs[-1], ys[-1], "s", color=DRONE_COLORS[n - 1], ms=8)
        tx, ty = cfg["target"]
        ax.plot(tx, ty, "x", color="#333333", ms=12, mew=2.5)
        ax.annotate("target", (tx, ty), textcoords="offset points", xytext=(8, 8),
                    fontsize=9, color="#333333")
        ax.set_title(f"{variant}: paths (latest run)  o = start, ■ = end", fontsize=10)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, color="#e6e6e6", lw=0.8)
        ax.legend(frameon=False, fontsize=9, loc="lower right")

    # --- Bottom row: every run, coloured by controller
    cfg = runs[-1]["meta"]["config"]
    ax_d, ax_f = axes[1]
    ax_d.axhline(cfg["desired_distance"], color="#666666", lw=1, ls="--")
    ax_d.text(0.5, cfg["desired_distance"] + 0.01, "desired spacing", fontsize=8, color="#666666")
    ax_d.axhline(cfg["protective_distance"], color="#999999", lw=1, ls=":")
    ax_d.text(0.5, cfg["protective_distance"] + 0.01, "safety push below", fontsize=8, color="#999999")
    for run in runs:
        c = VARIANT_COLORS[run["meta"]["variant"]]
        t = [r["t"] for r in run["rows"]]
        ax_d.plot(t, [min(r["d12"], r["d13"], r["d23"]) for r in run["rows"]], color=c, lw=2, alpha=0.7)
        ax_f.plot(t, [r["F_total"] for r in run["rows"]], color=c, lw=2, alpha=0.7)
    ax_d.set_title("Smallest distance between any two drones", fontsize=10)
    ax_d.set_ylabel("Distance (m)")
    ax_f.set_title("Total surprise of the swarm (target + neighbours)", fontsize=10)
    ax_f.set_ylabel("Surprise")
    ax_f.set_yscale("log")   # starts in the hundreds, ends near 1-20
    for ax in (ax_d, ax_f):
        ax.set_xlabel("Time (s)")
        ax.grid(True, color="#e6e6e6", lw=0.8)
        ax.spines[["top", "right"]].set_visible(False)

    handles = [plt.Line2D([], [], color=VARIANT_COLORS[v], lw=2, label=v) for v in VARIANTS]
    fig.suptitle(f"Three drones to a target ({args.mode}) — finished runs only", y=0.985, fontsize=12)
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.955), ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    output = f"analysis/figures/target_swarm_runs_{args.mode}.png"
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"Figure saved to {output}\n")

    print_table(group_metrics(runs, ["variant"], METRICS), ["variant"], METRICS)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
