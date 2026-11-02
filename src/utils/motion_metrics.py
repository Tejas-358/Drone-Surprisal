"""
motion_metrics.py — numbers about HOW drones moved, shared by the
second-order experiments (end-of-run metrics) and the analysis scripts
(which also apply them to older first-order runs, so all controllers are
compared with the same formulas).

  peak_speed     — largest commanded horizontal speed of any drone
  time_to_peak   — when that happened
  command_jerk   — RMS of how fast the velocity command changes (m/s^2);
                   small = smooth, large = jerky
  fitts_id       — Fitts' index of difficulty  log2(D / W + 1)
  fitts_fit      — least-squares line  MT = a + b * ID,  plus R^2

Used by: src/experiments/second_order_*_experiment.py, analysis/plot_second_order_*.py
"""

import math


def speed_series(rows, n_drones):
    """Per-step list of commanded speeds, one list per drone (keys vx1/vy1, ...)."""
    return [[math.hypot(r[f"vx{n}"], r[f"vy{n}"]) for r in rows]
            for n in range(1, n_drones + 1)]


def peak_speed(rows, n_drones):
    """(peak commanded speed in m/s, time of that peak in s)."""
    best, when = 0.0, 0.0
    for n in range(1, n_drones + 1):
        for r in rows:
            s = math.hypot(r[f"vx{n}"], r[f"vy{n}"])
            if s > best:
                best, when = s, r["t"]
    return best, when


def command_jerk(rows, n_drones, dt):
    """RMS over drones and steps of |v_cmd(k+1) - v_cmd(k)| / dt  (m/s^2)."""
    total, count = 0.0, 0
    for n in range(1, n_drones + 1):
        for a, b in zip(rows, rows[1:]):
            change = math.hypot(b[f"vx{n}"] - a[f"vx{n}"], b[f"vy{n}"] - a[f"vy{n}"]) / dt
            total += change * change
            count += 1
    return math.sqrt(total / count) if count else 0.0


def fitts_id(distance, width):
    """Fitts' index of difficulty (bits):  log2(D / W + 1)."""
    return math.log2(abs(distance) / width + 1.0)


def fitts_fit(ids, times):
    """
    Least-squares line  MT = a + b * ID.
    Output: (a, b, r2), or None if fewer than 2 distinct IDs.
    """
    n = len(ids)
    if n < 2 or len(set(round(i, 9) for i in ids)) < 2:
        return None
    mx, my = sum(ids) / n, sum(times) / n
    sxx = sum((x - mx) ** 2 for x in ids)
    sxy = sum((x - mx) * (y - my) for x, y in zip(ids, times))
    b = sxy / sxx
    a = my - b * mx
    ss_tot = sum((y - my) ** 2 for y in times)
    ss_res = sum((y - (a + b * x)) ** 2 for x, y in zip(ids, times))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return a, b, r2
