"""
run_log.py — save one run as a CSV plus a JSON "label" (metadata sidecar).

Every experiment saves its data the same way, so one analysis script can
find and compare runs:

    data/raw/<experiment>_<variant>_<mode>_<YYYYMMDD-HHMMSS>.csv    one row per step
    data/raw/<experiment>_<variant>_<mode>_<YYYYMMDD-HHMMSS>.json   how it was run

The JSON records everything needed to understand / repeat the run:
experiment, variant (controller), mode (sim/real), scenario, noise level,
random seed, whether the run was aborted, the git commit, the full config,
and the result metrics (if the experiment computes them).

Used by: every experiment in src/experiments/, at the end of a run (also
after an abort, so partial data is never lost).
"""

import csv
import json
import os
import subprocess
import time


def git_commit():
    """The current git commit hash, or "unknown" if git is not available."""
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"],
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return "unknown"


def save_run(rows, config, experiment, variant, meta):
    """
    Write the step-by-step CSV and its JSON sidecar.

    Input:  rows       — list of dictionaries, one per control step
            config     — the full config used (saved into the JSON)
            experiment — e.g. "target_swarm"
            variant    — e.g. "surprise" or "fixed" (the controller)
            meta       — extra facts for the JSON: scenario, noise_std, seed,
                         aborted, metrics, ...
    Output: path of the CSV that was written.

    Called from: the end of every experiment's main().
    """
    os.makedirs(config["log_dir"], exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    base = os.path.join(config["log_dir"], f"{experiment}_{variant}_{config['mode']}_{stamp}")

    with open(base + ".csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    sidecar = {
        "experiment": experiment,
        "variant": variant,
        "mode": config["mode"],
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "git_commit": git_commit(),
        "csv": os.path.basename(base + ".csv"),
        "n_rows": len(rows),
        **meta,
        "config": config,
    }
    with open(base + ".json", "w") as f:
        json.dump(sidecar, f, indent=2, default=str)

    print(f"\nSaved {len(rows)} rows to {base}.csv (+ .json)")
    return base + ".csv"
