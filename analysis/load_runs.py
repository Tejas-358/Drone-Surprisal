"""
load_runs.py — find and load runs from data/raw/ using their JSON sidecars.

Every experiment saves <name>.csv + <name>.json (see src/utils/run_log.py).
The JSON says which experiment, controller (variant), mode, scenario, noise,
seed, whether the run was aborted, and its metrics. This file reads them
back so every analysis script can say e.g. "all finished sim runs of the
target-swarm experiment".

Used by: the plot_*.py scripts in this folder.
"""

import csv
import glob
import json
import os


RAW_DIR = "data/raw"


def load_runs(experiment, mode=None, include_aborted=False):
    """
    Load all runs of one experiment.

    Input:  experiment      — "goal_navigation", "convoy", "formation" or "target_swarm"
            mode            — "sim", "real" or None (both)
            include_aborted — False (default): skip runs that stopped early
    Output: list of dictionaries, oldest first:
            {"meta": <the JSON sidecar>, "rows": <list of CSV rows, numbers as float>}
    """
    runs = []
    for path in sorted(glob.glob(os.path.join(RAW_DIR, "*.json"))):
        with open(path) as f:
            meta = json.load(f)
        if meta.get("experiment") != experiment:
            continue
        if mode and meta.get("mode") != mode:
            continue
        if meta.get("aborted") and not include_aborted:
            continue
        csv_path = os.path.join(RAW_DIR, meta["csv"])
        if not os.path.exists(csv_path):
            print(f"Missing CSV for {path}, skipped")
            continue
        runs.append({"meta": meta, "rows": read_csv(csv_path)})
    return runs


def read_csv(path):
    """Read a run CSV; turn numbers into floats and "True"/"False" into bools."""
    rows = []
    with open(path) as f:
        for raw in csv.DictReader(f):
            row = {}
            for key, value in raw.items():
                if value in ("True", "False"):
                    row[key] = value == "True"
                else:
                    try:
                        row[key] = float(value)
                    except ValueError:
                        row[key] = value
            rows.append(row)
    return rows


def group_metrics(runs, group_keys, metric_names):
    """
    Average each metric per group (e.g. per variant and scenario).

    Input:  runs         — from load_runs()
            group_keys   — sidecar fields to group by, e.g. ["variant", "scenario"]
            metric_names — which metrics to average
    Output: list of (group tuple, number of runs, {metric: (mean, std)})
            A metric that is empty in a run (e.g. "never settled") is left out
            of that metric's average; if it is empty in every run -> None.
    """
    groups = {}
    for run in runs:
        key = tuple(run["meta"].get(k, "") for k in group_keys)
        groups.setdefault(key, []).append(run["meta"].get("metrics", {}))

    table = []
    for key in sorted(groups):
        stats = {}
        for m in metric_names:
            values = [float(r[m]) for r in groups[key] if r.get(m, "") != ""]
            if values:
                mean = sum(values) / len(values)
                std = (sum((v - mean) ** 2 for v in values) / len(values)) ** 0.5
                stats[m] = (mean, std)
            else:
                stats[m] = None
        table.append((key, len(groups[key]), stats))
    return table


def print_table(table, group_keys, metric_names):
    """Print the output of group_metrics() as a readable table (mean +/- std)."""
    header = " ".join(f"{k:>12s}" for k in group_keys) + f" {'runs':>5s} " + \
        " ".join(f"{m:>22s}" for m in metric_names)
    print(header)
    print("-" * len(header))
    for key, n, stats in table:
        cells = []
        for m in metric_names:
            s = stats[m]
            cells.append(f"{'n/a':>22s}" if s is None else f"{s[0]:>12.3f} +/- {s[1]:<6.3f}")
        print(" ".join(f"{str(k):>12s}" for k in key) + f" {n:5d} " + " ".join(cells))
