"""
plot_convoy_runs.py — compare the convoy experiments E1 (surprise-gated)
and E2 (baseline) from all finished runs in data/raw/.

Figure (analysis/figures/convoy_runs_<mode>.png):
  top    — the smaller of the two follower gaps over time, every run, with
           the target gap (follow_distance) and the safety distance
  bottom — the smaller follower speed factor exp(-k*U) over time
           (1 = moving normally, near 0 = frozen by surprise). For E2 it is
           computed but not used, so only E1's curve affects the flight.
Colour = variant (blue = baseline E2, orange = surprise E1).

The table groups runs by variant AND noise_scale (N), so a sweep over N
(H2: "for which N does gating help without freezing?") shows directly:
    low_factor_fraction high  -> the followers were mostly frozen
    overshoot_after_stop low  -> the convoy did not bunch up after the stop

Run from the repo root:
    python -m analysis.plot_convoy_runs --mode sim
"""

import argparse
import os

import matplotlib
import matplotlib.pyplot as plt

from analysis.load_runs import load_runs, group_metrics, print_table


COLORS = {"baseline": "#2a78d6", "surprise": "#eb6834"}
METRICS = ["min_gap", "overshoot_after_stop", "settling_after_stop",
           "low_factor_fraction", "repel_fraction"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parser.add_argument("--show", action="store_true", help="open a window as well")
    args = parser.parse_args()
    if not args.show:
        matplotlib.use("Agg")

    runs = load_runs("convoy", args.mode)
    print(f"{len(runs)} finished {args.mode} run(s) found.")
    if not runs:
        return

    # Make N (noise_scale) a top-level field so we can group by it.
    for run in runs:
        run["meta"]["N"] = run["meta"]["config"]["noise_scale"]

    fig, (ax_g, ax_f) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    cfg = runs[-1]["meta"]["config"]
    ax_g.axhline(cfg["follow_distance"], color="#666666", lw=1, ls="--")
    ax_g.text(0.3, cfg["follow_distance"] + 0.01, "target gap", fontsize=8, color="#666666")
    ax_g.axhline(cfg["safe_distance"], color="#999999", lw=1, ls=":")
    ax_g.text(0.3, cfg["safe_distance"] + 0.01, "safety push below", fontsize=8, color="#999999")

    for run in runs:
        c = COLORS[run["meta"]["variant"]]
        rows = run["rows"]
        t = [r["t"] for r in rows]
        ax_g.plot(t, [min(r["gap_12"], r["gap_23"]) for r in rows], color=c, lw=2, alpha=0.7)
        ax_f.plot(t, [min(r["factor2"], r["factor3"]) for r in rows], color=c, lw=2, alpha=0.7)
        stop = run["meta"]["metrics"].get("stop_time", "")
        if stop != "":
            ax_g.axvline(float(stop), color=c, lw=0.8, ls=":", alpha=0.6)

    ax_g.set_title("Smaller follower gap (dotted vertical line = leader stopped)", fontsize=10)
    ax_g.set_ylabel("Gap (m)")
    ax_f.set_title("Smaller follower speed factor exp(-k U)  (only E1 uses it)", fontsize=10)
    ax_f.set_ylabel("Speed factor")
    ax_f.set_ylim(-0.05, 1.05)
    ax_f.set_xlabel("Time (s)")
    for ax in (ax_g, ax_f):
        ax.grid(True, color="#e6e6e6", lw=0.8)
        ax.spines[["top", "right"]].set_visible(False)

    handles = [plt.Line2D([], [], color=COLORS[v], lw=2,
                          label={"baseline": "E2 baseline", "surprise": "E1 surprise"}[v])
               for v in ("baseline", "surprise")]
    fig.suptitle(f"Convoy E1 vs E2 ({args.mode}) — finished runs only", y=0.985, fontsize=12)
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.95), ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.91))

    output = f"analysis/figures/convoy_runs_{args.mode}.png"
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"Figure saved to {output}\n")

    print_table(group_metrics(runs, ["variant", "N"], METRICS), ["variant", "N"], METRICS)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
