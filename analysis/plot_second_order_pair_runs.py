"""
plot_second_order_pair_runs.py — results of the two-drone second-order
experiment (EXPERIMENTS.md §7), compared with the first-order two-drone runs
(fixed / adaptive) of the same mode.

Figure (analysis/figures/second_order_pair_runs_<mode>.png):
  top row      — distance between the drones over time, "too far" (left) and
                 "too close" (right) starts; colour = controller
  bottom left  — Fitts sweep: commanded drone speed over time, one line per
                 start gap (bell-shaped profile expected)
  bottom right — Fitts plot: settling time against ID = log2(D / W + 1),
                 with the least-squares line MT = a + b ID and its R^2
It also prints mean +/- std of the metrics per controller and scenario,
and the Fitts fit. Smoothness metrics (peak_speed, command_jerk) are
recomputed from the CSVs for the first-order runs, so every controller is
measured with the same formulas.

Run from the repo root:
    python -m analysis.plot_second_order_pair_runs --mode sim
    python -m analysis.plot_second_order_pair_runs --mode sim --show
"""

import argparse
import math
import os

import matplotlib
import matplotlib.pyplot as plt

from analysis.load_runs import group_metrics, load_runs, print_table
from src.utils.motion_metrics import command_jerk, fitts_fit, fitts_id, peak_speed


VARIANTS = ["fixed", "adaptive", "second_order", "second_order_adaptive"]
COLORS = {"fixed": "#2a78d6", "adaptive": "#eb6834",
          "second_order": "#1baf7a", "second_order_adaptive": "#8a4fd8"}
METRICS = ["settling_time", "overshoot", "rms_error", "min_distance",
           "peak_speed", "command_jerk"]
SCENARIOS = ["too_far", "too_close"]


def first_order_runs(mode):
    """
    First-order two-drone runs, with their velocity columns renamed to the
    vx1/vy1 convention and the smoothness metrics added.
    """
    runs = load_runs("goal_navigation", mode)
    for run in runs:
        for r in run["rows"]:
            for n in (1, 2):
                r[f"vx{n}"], r[f"vy{n}"] = r[f"v{n}x"], r[f"v{n}y"]
        dt = run["meta"]["config"]["control_period"]
        peak, t_peak = peak_speed(run["rows"], 2)
        run["meta"]["metrics"] = {**run["meta"].get("metrics", {}),
                                  "peak_speed": round(peak, 3),
                                  "time_to_peak": round(t_peak, 2),
                                  "command_jerk": round(command_jerk(run["rows"], 2, dt), 4)}
    return runs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="sim", choices=["sim", "real"])
    parser.add_argument("--show", action="store_true", help="open a window as well")
    args = parser.parse_args()
    if not args.show:
        matplotlib.use("Agg")

    second = load_runs("second_order_pair", args.mode)
    first = first_order_runs(args.mode)
    print(f"{len(second)} second-order and {len(first)} first-order finished {args.mode} run(s).")
    if not second and not first:
        return

    compare = [r for r in first + second if r["meta"].get("scenario") in SCENARIOS]
    sweep = [r for r in second if r["meta"].get("start_gap") is not None]

    fig, axes = plt.subplots(2, 2, figsize=(11.5, 8.5))

    # --- Top row: distance over time per scenario, every controller
    for col, scenario in enumerate(SCENARIOS):
        ax = axes[0][col]
        mine = [r for r in compare if r["meta"]["scenario"] == scenario]
        if mine:
            cfg = mine[-1]["meta"]["config"]
            target, tol = cfg["desired_distance"], cfg["distance_tolerance"]
            ax.axhspan(target - tol, target + tol, color="#000000", alpha=0.06, lw=0)
            ax.axhline(target, color="#666666", lw=1, ls="--")
        for r in mine:
            ax.plot([x["t"] for x in r["rows"]], [x["distance"] for x in r["rows"]],
                    color=COLORS[r["meta"]["variant"]], lw=2, alpha=0.8)
        ax.set_title(f"{scenario.replace('_', ' ').capitalize()} start ({len(mine)} runs)",
                     fontsize=10)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance between drones (m)")

    # --- Bottom left: speed profiles of the Fitts sweep
    ax_v, ax_f = axes[1]
    gaps = sorted({r["meta"]["start_gap"] for r in sweep})
    cmap = plt.get_cmap("viridis")
    for r in sweep:
        g = r["meta"]["start_gap"]
        shade = cmap(gaps.index(g) / max(len(gaps) - 1, 1))
        t = [x["t"] for x in r["rows"]]
        ax_v.plot(t, [max(math.hypot(x["vx1"], x["vy1"]), math.hypot(x["vx2"], x["vy2"]))
                      for x in r["rows"]], color=shade, lw=1.8, alpha=0.8)
    for i, g in enumerate(gaps):
        ax_v.plot([], [], color=cmap(i / max(len(gaps) - 1, 1)), lw=2, label=f"start {g:.2f} m")
    ax_v.set_title("Fitts sweep: commanded drone speed", fontsize=10)
    ax_v.set_xlabel("Time (s)")
    ax_v.set_ylabel("Speed (m/s)")
    if gaps:
        ax_v.legend(frameon=False, fontsize=8)

    # --- Bottom right: Fitts plot
    fit_text = "no sweep runs yet"
    points = []
    for r in sweep:
        m = r["meta"]["metrics"]
        if m.get("settling_time", "") != "":
            points.append((float(m["fitts_id"]), float(m["settling_time"]),
                           r["meta"]["variant"]))
    for variant in ("second_order", "second_order_adaptive"):
        pts = [(x, y) for x, y, v in points if v == variant]
        if not pts:
            continue
        ax_f.plot([p[0] for p in pts], [p[1] for p in pts], "o",
                  color=COLORS[variant], ms=7, label=variant)
        fit = fitts_fit([p[0] for p in pts], [p[1] for p in pts])
        if fit:
            a, b, r2 = fit
            xs = [min(p[0] for p in pts), max(p[0] for p in pts)]
            ax_f.plot(xs, [a + b * x for x in xs], color=COLORS[variant], lw=1.5, ls="--")
            fit_text = f"{variant}: MT = {a:.2f} + {b:.2f}·ID   R² = {r2:.2f}"
            print(f"Fitts fit ({variant}, {len(pts)} runs): a = {a:.3f} s, "
                  f"b = {b:.3f} s/bit, R^2 = {r2:.3f}")
    ax_f.set_title("Fitts' law: settling time vs index of difficulty", fontsize=10)
    ax_f.set_xlabel("ID = log2(D / W + 1)  (bits)")
    ax_f.set_ylabel("Settling time (s)")
    ax_f.text(0.02, 0.96, fit_text, transform=ax_f.transAxes, fontsize=8,
              va="top", color="#333333")
    if points:
        ax_f.legend(frameon=False, fontsize=8, loc="lower right")

    for row in axes:
        for ax in row:
            ax.grid(True, color="#e6e6e6", lw=0.8)
            ax.spines[["top", "right"]].set_visible(False)

    handles = [plt.Line2D([], [], color=COLORS[v], lw=2, label=v) for v in VARIANTS]
    fig.suptitle(f"Two drones, first vs second order ({args.mode}) — finished runs only",
                 y=0.985, fontsize=12)
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.955),
               ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    output = f"analysis/figures/second_order_pair_runs_{args.mode}.png"
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"Figure saved to {output}\n")

    keys = ["variant", "scenario"]
    print_table(group_metrics(compare, keys, METRICS), keys, METRICS)
    if sweep:
        print("\nFitts sweep:")
        print_table(group_metrics(sweep, ["variant", "start_gap"],
                                  ["fitts_id", "settling_time", "peak_speed"]),
                    ["variant", "start_gap"], ["fitts_id", "settling_time", "peak_speed"])
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
