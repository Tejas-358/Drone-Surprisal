"""
plot_formation_runs.py — results of E3 (formation-distance gradient
descent) from all finished runs in data/raw/.

Figure (analysis/figures/formation_runs_<mode>.png):
  top    — both neighbour gaps over time, every run, with the target gap
  bottom — formation potential U over time (0 = perfect formation)
Hypothesis H3: the gaps converge to the target and U goes to ~0.

Run from the repo root:
    python -m analysis.plot_formation_runs --mode sim
"""

import argparse
import os

import matplotlib
import matplotlib.pyplot as plt

from analysis.load_runs import load_runs, group_metrics, print_table


GAP_COLORS = {"gap_12": "#2a78d6", "gap_23": "#eb6834"}
METRICS = ["convergence_time", "final_gap_error", "final_U", "min_gap"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parser.add_argument("--show", action="store_true", help="open a window as well")
    args = parser.parse_args()
    if not args.show:
        matplotlib.use("Agg")

    runs = load_runs("formation", args.mode)
    print(f"{len(runs)} finished {args.mode} run(s) found.")
    if not runs:
        return

    fig, (ax_g, ax_u) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    target = runs[-1]["meta"]["config"]["desired_distance"]
    ax_g.axhline(target, color="#666666", lw=1, ls="--")
    ax_g.text(0.2, target + 0.01, "target gap", fontsize=8, color="#666666")

    for run in runs:
        rows = run["rows"]
        t = [r["t"] for r in rows]
        for gap, c in GAP_COLORS.items():
            ax_g.plot(t, [r[gap] for r in rows], color=c, lw=2, alpha=0.7)
        ax_u.plot(t, [r["U"] for r in rows], color="#444444", lw=2, alpha=0.7)

    ax_g.set_title("Neighbour gaps", fontsize=10)
    ax_g.set_ylabel("Gap (m)")
    ax_u.set_title("Formation potential U = 1/2 (S12^2 + S23^2)", fontsize=10)
    ax_u.set_ylabel("U (m^2)")
    ax_u.set_xlabel("Time (s)")
    for ax in (ax_g, ax_u):
        ax.grid(True, color="#e6e6e6", lw=0.8)
        ax.spines[["top", "right"]].set_visible(False)

    handles = [plt.Line2D([], [], color=c, lw=2, label=g.replace("gap_", "gap CF").replace("CF12", "CF1-CF2")
                          .replace("CF23", "CF2-CF3")) for g, c in GAP_COLORS.items()]
    fig.suptitle(f"E3 formation distance ({args.mode}) — finished runs only", y=0.985, fontsize=12)
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.95), ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.91))

    output = f"analysis/figures/formation_runs_{args.mode}.png"
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"Figure saved to {output}\n")

    print_table(group_metrics(runs, ["variant"], METRICS), ["variant"], METRICS)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
