"""
summarize_sim_runs.py — the numbers behind document/simulation-results.md.

Reads the JSON + CSV of every finished simulator run and prints tables per
experiment and condition. By default it uses only runs from the batch that
started on 2 Nov (--since 20261102): the earlier 28 Oct batch was partial.
Use --since 0 to include everything and see how much that changes.

It adds one metric the experiments do not save:
  hold_time — first time the gap stays within +-8 cm of the target for 2 s.
              Noise makes "never leaves the band again" (settling_time)
              unreliable; this measures "arrived and stayed for a while".

Run from the repo root:
    python -m analysis.summarize_sim_runs
    python -m analysis.summarize_sim_runs --since 0
"""

import argparse

from analysis.load_runs import load_runs


def stamp(run):
    """The run's start date and time as a number-like string, e.g. 20261102-103345."""
    return run["meta"]["csv"].rsplit(".", 1)[0][-15:]


def mean_std(values):
    """Text 'mean +/- std (n)' for a list of numbers; empty values are left out."""
    values = [float(v) for v in values if v not in ("", None)]
    if not values:
        return "n/a"
    mean = sum(values) / len(values)
    std = (sum((v - mean) ** 2 for v in values) / len(values)) ** 0.5
    return f"{mean:.3f} +/- {std:.3f} (n={len(values)})"


def hold_time(rows, target, tolerance=0.08, hold=2.0):
    """First time |distance - target| stays <= tolerance for `hold` seconds, or None."""
    for i, row in enumerate(rows):
        if row["t"] + hold > rows[-1]["t"]:
            return None
        window = [r for r in rows[i:] if r["t"] <= row["t"] + hold]
        if all(abs(r["distance"] - target) <= tolerance for r in window):
            return row["t"]
    return None


def two_drones(since):
    print("\n=== Two drones (goal_navigation) ===")
    runs = [r for r in load_runs("goal_navigation", "sim") if stamp(r) >= since]
    for scenario in ("too_far", "too_close"):
        for controller in ("fixed", "adaptive"):
            group = [r for r in runs if r["meta"]["scenario"] == scenario
                     and r["meta"]["variant"] == controller]
            print(f"\n{scenario} / {controller}: {len(group)} runs")
            for name in ("rms_error", "overshoot", "min_distance", "settling_time"):
                print(f"  {name:15s} {mean_std([r['meta']['metrics'][name] for r in group])}")
            holds = [hold_time(r["rows"], 0.6) for r in group]
            print(f"  {'hold_time':15s} {mean_std(holds)}  "
                  f"never held: {sum(h is None for h in holds)}")
            print(f"  per seed: " + ", ".join(
                f"{r['meta']['seed']}: start {r['meta']['metrics']['start_distance']:.2f} "
                f"rms {r['meta']['metrics']['rms_error']:.3f} "
                f"hold {hold_time(r['rows'], 0.6)}" for r in group))


def swarm(since):
    print("\n=== Three drones (target_swarm) ===")
    runs = [r for r in load_runs("target_swarm", "sim") if stamp(r) >= since]
    for variant in ("goal_only", "surprise"):
        group = [r for r in runs if r["meta"]["variant"] == variant]
        print(f"\n{variant}: {len(group)} runs, seeds {[r['meta']['seed'] for r in group]}")
        for name in ("arrival_time", "min_pair_distance", "final_spacing_error",
                     "protective_fraction", "final_total_surprise"):
            print(f"  {name:22s} {mean_std([r['meta']['metrics'][name] for r in group])}")


def convoy(since):
    print("\n=== Convoy (E2 baseline vs E1 surprise, per N) ===")
    runs = [r for r in load_runs("convoy", "sim") if stamp(r) >= since]
    conditions = [("baseline", None)] + [("surprise", n) for n in (0.05, 0.1, 0.2, 0.4)]
    for variant, n in conditions:
        group = [r for r in runs if r["meta"]["variant"] == variant
                 and (n is None or r["meta"]["config"]["noise_scale"] == n)]
        print(f"\n{variant} N={n}: {len(group)} runs")
        for name in ("overshoot_after_stop", "min_gap", "settling_after_stop",
                     "final_gap_error", "low_factor_fraction", "repel_fraction"):
            values = [r["meta"]["metrics"][name] for r in group]
            print(f"  {name:22s} {mean_std(values)}")
        print(f"  {'final gap CF1-CF2':22s} "
              f"{mean_std([r['rows'][-1]['gap_12'] for r in group])}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--since", default="20261102",
                        help="use runs from this date on (YYYYMMDD); 0 = all runs")
    args = parser.parse_args()
    two_drones(args.since)
    swarm(args.since)
    convoy(args.since)


if __name__ == "__main__":
    main()
