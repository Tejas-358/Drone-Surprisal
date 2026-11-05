"""
summarize_second_order_sim.py — the second-order numbers behind
document/simulation-results.md (batch on commit 3106f2f, 4-5 Nov).

That batch ran on a machine where telemetry stalled often, so every table
shows two rows of evidence: all finished runs, and only the "clean" ones
(no gap above 0.5 s between control steps).

It compares first and second order under the same conditions (same batch,
same start layout, same noise), and fits Fitts' law both ways:
    log fit:    MT = a + b * ID,  ID = log2(D / W + 1)
    linear fit: MT = a + b * D    (what a constant speed limit would give)

Run from the repo root:
    python -m analysis.summarize_second_order_sim
"""

import argparse

from analysis.load_runs import load_runs
from analysis.summarize_sim_runs import hold_time, mean_std
from src.utils.motion_metrics import fitts_fit


def max_step_gap(rows):
    """Longest time between two control steps (s)."""
    times = [r["t"] for r in rows]
    return max(b - a for a, b in zip(times, times[1:]))


def batch(experiment, commit):
    """Finished sim runs of one experiment on one commit."""
    return [r for r in load_runs(experiment, "sim") if r["meta"]["git_commit"] == commit]


def clean(runs):
    return [r for r in runs if max_step_gap(r["rows"]) <= 0.5]


def jerk(rows, n_drones):
    """RMS change of the velocity command per second; handles both column styles."""
    total, count = 0.0, 0
    for n in range(1, n_drones + 1):
        names = (f"vx{n}", f"vy{n}") if f"vx{n}" in rows[0] else (f"v{n}x", f"v{n}y")
        for a, b in zip(rows, rows[1:]):
            dt = max(b["t"] - a["t"], 1e-3)
            total += ((b[names[0]] - a[names[0]]) ** 2 + (b[names[1]] - a[names[1]]) ** 2) / dt / dt
            count += 1
    return (total / count) ** 0.5


def final_spacing(rows):
    """Mean pair distance over the last 5 s of a three-drone run."""
    tail = [r for r in rows if r["t"] >= rows[-1]["t"] - 5.0]
    return sum((r["d12"] + r["d13"] + r["d23"]) / 3 for r in tail) / len(tail)


def pair_table(commit):
    print("\n=== Two drones, same batch ===")
    groups = [("goal_navigation", "fixed"), ("goal_navigation", "adaptive"),
              ("second_order_pair", "second_order"), ("second_order_pair", "second_order_adaptive")]
    for scenario in ("too_far", "too_close"):
        for experiment, variant in groups:
            runs = [r for r in batch(experiment, commit)
                    if r["meta"]["variant"] == variant and r["meta"]["scenario"] == scenario]
            for label, group in (("all", runs), ("clean", clean(runs))):
                holds = [hold_time(r["rows"], 0.6) for r in group]
                print(f"{scenario:9s} {variant:22s} {label:5s} n={len(group)} "
                      f"hold {mean_std(holds)} never={sum(h is None for h in holds)} | "
                      f"rms {mean_std([r['meta']['metrics']['rms_error'] for r in group])} | "
                      f"jerk {mean_std([jerk(r['rows'], 2) for r in group])}")


def swarm_table(commit):
    print("\n=== Three drones, same batch (default target) ===")
    groups = [("target_swarm", "goal_only"), ("target_swarm", "surprise"),
              ("second_order_swarm", "goal_only_second_order"),
              ("second_order_swarm", "second_order")]
    for experiment, variant in groups:
        runs = [r for r in batch(experiment, commit) if r["meta"]["variant"] == variant
                and r["meta"].get("target_distance") is None]
        print(f"\n{variant}: n={len(runs)} (clean {len(clean(runs))}), "
              f"seeds {[r['meta']['seed'] for r in runs]}, "
              f"speed limit {runs[0]['meta']['config']['max_horizontal_speed']}, "
              f"start centre error {mean_std([r['rows'][0]['centre_error'] for r in runs])}")
        for name in ("arrival_time", "min_pair_distance", "final_spacing_error",
                     "protective_fraction"):
            print(f"  {name:22s} {mean_std([r['meta']['metrics'][name] for r in runs])}")
        print(f"  {'command_jerk':22s} {mean_std([jerk(r['rows'], 3) for r in runs])}")
        print(f"  {'final mean spacing':22s} {mean_std([final_spacing(r['rows']) for r in runs])}")


def show_fit(label, xs, times, name):
    fit = fitts_fit(xs, times)
    if fit:
        print(f"  {label:26s} MT = {fit[0]:.2f} + {fit[1]:.2f} * {name}   R2 = {fit[2]:.3f}  n={len(xs)}")


def fitts(commit):
    print("\n=== Fitts sweeps ===")
    runs = [r for r in batch("second_order_swarm", commit)
            if r["meta"]["variant"] == "second_order"
            and r["meta"]["metrics"].get("arrival_time", "") != ""]
    points = sorted((r["meta"]["metrics"]["start_centre_error"],
                     r["meta"]["metrics"]["fitts_id"],
                     float(r["meta"]["metrics"]["arrival_time"])) for r in runs)
    print("Swarm (D, ID, arrival s):", [(round(d, 2), round(i, 2), t) for d, i, t in points])
    for label, pts in (("all distances", points), ("D <= 0.8", [p for p in points if p[0] <= 0.85])):
        show_fit(label + ", log", [p[1] for p in pts], [p[2] for p in pts], "ID")
        show_fit(label + ", linear", [p[0] for p in pts], [p[2] for p in pts], "D")

    runs = [r for r in batch("second_order_pair", commit)
            if r["meta"]["variant"] == "second_order"
            and r["meta"]["scenario"].startswith("gap_")]
    points = []
    for r in runs:
        h = hold_time(r["rows"], 0.6)
        if h is not None:
            d = r["meta"]["metrics"]["start_distance"]
            points.append((d - 0.6, r["meta"]["metrics"]["fitts_id"], h))
    points.sort()
    print("Pair (gap error, ID, hold s):", [(round(d, 2), round(i, 2), round(t, 2)) for d, i, t in points])
    show_fit("log", [p[1] for p in points], [p[2] for p in points], "ID")
    show_fit("linear", [p[0] for p in points], [p[2] for p in points], "D")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--commit", default="3106f2f", help="git commit of the batch")
    args = parser.parse_args()
    pair_table(args.commit)
    swarm_table(args.commit)
    fitts(args.commit)


if __name__ == "__main__":
    main()
