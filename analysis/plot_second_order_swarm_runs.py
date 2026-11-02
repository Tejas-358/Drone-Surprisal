"""
plot_second_order_swarm_runs.py — results of the three-drone second-order
swarm (EXPERIMENTS.md §8), compared with the first-order target-swarm runs
(goal_only / surprise) of the same mode.

Figure (analysis/figures/second_order_swarm_runs_<mode>.png):
  top left     — top-down paths of the latest second_order run (default target)
  top right    — smallest pair distance over time, every comparison run;
                 colour = controller
  bottom left  — Fitts sweep: swarm-centre speed over time, one line per
                 target distance (bell shape, flattened by the speed limit)
  bottom right — Fitts plot: arrival time against ID = log2(D / W + 1), with
                 one fit on D <= fitts_max_distance (below the speed limit)
                 and one on all distances
It also prints mean +/- std of the metrics per controller and the Fitts fits.
Smoothness metrics are recomputed from the CSVs for the first-order runs.

Run from the repo root:
    python -m analysis.plot_second_order_swarm_runs --mode sim
    python -m analysis.plot_second_order_swarm_runs --mode sim --show
"""

import argparse
import os

import matplotlib
import matplotlib.pyplot as plt

from analysis.load_runs import group_metrics, load_runs, print_table
from src.utils.motion_metrics import command_jerk, fitts_fit, peak_speed


VARIANTS = ["goal_only", "surprise", "goal_only_second_order", "second_order"]
COLORS = {"goal_only": "#2a78d6", "surprise": "#eb6834",
          "goal_only_second_order": "#7fb3e8", "second_order": "#1baf7a"}
DRONE_COLORS = ["#1baf7a", "#eda100", "#e87ba4"]
METRICS = ["arrival_time", "min_pair_distance", "final_spacing_error",
           "protective_fraction", "peak_speed", "command_jerk"]


def first_order_runs(mode):
    """First-order target-swarm runs with the smoothness metrics added."""
    runs = load_runs("target_swarm", mode)
    for run in runs:
        dt = run["meta"]["config"]["control_period"]
        peak, t_peak = peak_speed(run["rows"], 3)
        run["meta"]["metrics"] = {**run["meta"].get("metrics", {}),
                                  "peak_speed": round(peak, 3),
                                  "time_to_peak": round(t_peak, 2),
                                  "command_jerk": round(command_jerk(run["rows"], 3, dt), 4)}
    return runs


def plot_fit(ax, pts, color, label, style):
    """Least-squares Fitts line through pts; returns a one-line summary."""
    fit = fitts_fit([p[0] for p in pts], [p[1] for p in pts])
    if not fit:
        return f"{label}: not enough distances"
    a, b, r2 = fit
    xs = [min(p[0] for p in pts), max(p[0] for p in pts)]
    ax.plot(xs, [a + b * x for x in xs], color=color, lw=1.5, ls=style)
    return f"{label}: MT = {a:.2f} + {b:.2f}·ID   R² = {r2:.2f}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parser.add_argument("--fitts-max-distance", type=float, default=0.8,
                        help="largest D (m) expected below the speed limit")
    parser.add_argument("--show", action="store_true", help="open a window as well")
    args = parser.parse_args()
    if not args.show:
        matplotlib.use("Agg")

    second = load_runs("second_order_swarm", args.mode)
    first = first_order_runs(args.mode)
    print(f"{len(second)} second-order and {len(first)} first-order finished {args.mode} run(s).")
    if not second and not first:
        return

    compare = first + [r for r in second if r["meta"].get("target_distance") is None]
    sweep = [r for r in second if r["meta"].get("target_distance") is not None]

    fig, axes = plt.subplots(2, 2, figsize=(11.5, 8.5))
    ax_p, ax_d = axes[0]
    ax_v, ax_f = axes[1]

    # --- Top left: paths of the latest default-target second_order run
    latest = [r for r in compare if r["meta"]["variant"] == "second_order"]
    if latest:
        rows, cfg = latest[-1]["rows"], latest[-1]["meta"]["config"]
        for n in (1, 2, 3):
            xs, ys = [r[f"x{n}"] for r in rows], [r[f"y{n}"] for r in rows]
            ax_p.plot(xs, ys, color=DRONE_COLORS[n - 1], lw=2, label=f"CF{n}")
            ax_p.plot(xs[0], ys[0], "o", color=DRONE_COLORS[n - 1], ms=8)
            ax_p.plot(xs[-1], ys[-1], "s", color=DRONE_COLORS[n - 1], ms=8)
        tx, ty = cfg["target"]
        ax_p.plot(tx, ty, "x", color="#333333", ms=12, mew=2.5)
        ax_p.set_aspect("equal", adjustable="datalim")
        ax_p.legend(frameon=False, fontsize=9, loc="lower right")
        ax_p.set_title("second_order: paths (latest run)  o = start, ■ = end", fontsize=10)
    else:
        ax_p.set_title("second_order: no runs yet", fontsize=10)
    ax_p.set_xlabel("x (m)")
    ax_p.set_ylabel("y (m)")

    # --- Top right: smallest pair distance, every comparison run
    if compare:
        cfg = compare[-1]["meta"]["config"]
        ax_d.axhline(cfg["desired_distance"], color="#666666", lw=1, ls="--")
        ax_d.axhline(cfg["protective_distance"], color="#999999", lw=1, ls=":")
    for run in compare:
        rows = run["rows"]
        ax_d.plot([r["t"] for r in rows], [min(r["d12"], r["d13"], r["d23"]) for r in rows],
                  color=COLORS[run["meta"]["variant"]], lw=2, alpha=0.7)
    ax_d.set_title("Smallest distance between any two drones", fontsize=10)
    ax_d.set_xlabel("Time (s)")
    ax_d.set_ylabel("Distance (m)")

    # --- Bottom left: centre speed in the sweep
    dists = sorted({r["meta"]["target_distance"] for r in sweep})
    cmap = plt.get_cmap("viridis")
    for run in sweep:
        D = run["meta"]["target_distance"]
        shade = cmap(dists.index(D) / max(len(dists) - 1, 1))
        ax_v.plot([r["t"] for r in run["rows"]], [r["centre_speed"] for r in run["rows"]],
                  color=shade, lw=1.8, alpha=0.8)
    for i, D in enumerate(dists):
        ax_v.plot([], [], color=cmap(i / max(len(dists) - 1, 1)), lw=2, label=f"D = {D:.1f} m")
    if sweep:
        ax_v.axhline(sweep[-1]["meta"]["config"]["max_horizontal_speed"], color="#999999",
                     lw=1, ls=":")
        ax_v.legend(frameon=False, fontsize=8)
    ax_v.set_title("Fitts sweep: swarm-centre speed (dotted = speed limit)", fontsize=10)
    ax_v.set_xlabel("Time (s)")
    ax_v.set_ylabel("Speed (m/s)")

    # --- Bottom right: Fitts plot (second_order only)
    pts = []
    for run in sweep:
        m = run["meta"]["metrics"]
        if run["meta"]["variant"] == "second_order" and m.get("arrival_time", "") != "":
            pts.append((float(m["fitts_id"]), float(m["arrival_time"]),
                        float(m["start_centre_error"])))
    lines = ["no sweep runs yet"]
    if pts:
        near = [p for p in pts if p[2] <= args.fitts_max_distance]
        far = [p for p in pts if p[2] > args.fitts_max_distance]
        ax_f.plot([p[0] for p in near], [p[1] for p in near], "o", color=COLORS["second_order"],
                  ms=7, label=f"D ≤ {args.fitts_max_distance} m")
        ax_f.plot([p[0] for p in far], [p[1] for p in far], "o", mfc="none",
                  color=COLORS["second_order"], ms=7, label=f"D > {args.fitts_max_distance} m")
        lines = [plot_fit(ax_f, near, COLORS["second_order"], "near", "--"),
                 plot_fit(ax_f, pts, "#666666", "all", ":")]
        for line in lines:
            print("Fitts fit " + line)
        ax_f.legend(frameon=False, fontsize=8, loc="lower right")
    ax_f.text(0.02, 0.96, "\n".join(lines), transform=ax_f.transAxes, fontsize=8,
              va="top", color="#333333")
    ax_f.set_title("Fitts' law: arrival time vs index of difficulty", fontsize=10)
    ax_f.set_xlabel("ID = log2(D / W + 1)  (bits)")
    ax_f.set_ylabel("Arrival time (s)")

    for row in axes:
        for ax in row:
            ax.grid(True, color="#e6e6e6", lw=0.8)
            ax.spines[["top", "right"]].set_visible(False)

    handles = [plt.Line2D([], [], color=COLORS[v], lw=2, label=v) for v in VARIANTS]
    fig.suptitle(f"Three drones to a target, first vs second order ({args.mode}) — finished runs only",
                 y=0.985, fontsize=12)
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.955),
               ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    output = f"analysis/figures/second_order_swarm_runs_{args.mode}.png"
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"Figure saved to {output}\n")

    print_table(group_metrics(compare, ["variant"], METRICS), ["variant"], METRICS)
    if sweep:
        print("\nFitts sweep:")
        cols = ["fitts_id", "arrival_time", "peak_speed", "centre_overshoot"]
        print_table(group_metrics(sweep, ["variant", "target_distance"], cols),
                    ["variant", "target_distance"], cols)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
