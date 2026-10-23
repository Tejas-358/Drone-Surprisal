"""
target_swarm_controller.py — surprise minimisation for drones flying
TOGETHER to a target (no leader, nobody follows anybody).

Each drone carries two kinds of expectation:
  1. about the TARGET:        "I should be at the target point"
         surprise  S_goal = 1/2 * (|p - target| / sigma_goal)^2
  2. about EACH OTHER drone:  "my neighbour should be desired_distance away"
         surprise  S_ij   = 1/2 * ((d_ij - desired_distance) / sigma_distance)^2

  Total surprise of drone i:   F_i = S_goal + sum over neighbours j of S_ij

Each drone moves in the direction that lowers ITS OWN total surprise the
fastest (gradient descent on F_i), with a speed limit:
      v_i = - step_gain * gradient(F_i)

All drones cannot sit exactly on the target AND be 0.6 m apart, so the
surprises balance out: the drones settle in a ring (3 drones: a triangle)
around the target, spaced close to desired_distance.

Two variants, chosen per run:
  "surprise"  — both expectations (target + neighbours)       (PROPOSED)
  "goal_only" — only the target expectation; drones ignore    (BASELINE)
                each other, so they crowd towards the same point

In BOTH variants a safety push (not part of the surprise) moves a drone
away from any neighbour closer than protective_distance.

This file is pure maths (positions in, velocities out): it never talks to a
drone, so it runs unchanged in the simulator and on real drones.

Used by: src/experiments/target_swarm_experiment.py, once per control step.
"""

import math


def limit_speed(vx, vy, max_speed):
    """Shrink (vx, vy) to at most max_speed, keeping its direction."""
    speed = math.hypot(vx, vy)
    if speed > max_speed:
        return vx * max_speed / speed, vy * max_speed / speed
    return vx, vy


def swarm_step(positions, target, config, variant):
    """
    Compute one velocity command per drone.

    Input:  positions — list of (x, y) per drone, in metres (what the drones see)
            target    — (x, y) of the target point
            config    — uses sigma_goal, sigma_distance, desired_distance,
                        step_gain, max_horizontal_speed, protective_distance,
                        separation_recovery_gain
            variant   — "surprise" or "goal_only"
    Output: list of dictionaries, one per drone, with
            "vx", "vy"      — velocity command (m/s)
            "S_goal"        — this drone's surprise about the target
            "S_neighbours"  — sum of its surprises about the other drones
                              (computed for logging in BOTH variants)
            "F"             — total surprise it acts on (variant-dependent)
            "protective"    — True if the safety push was active
    """
    out = []
    for i, (xi, yi) in enumerate(positions):
        # --- Expectation 1: be at the target.
        gx, gy = xi - target[0], yi - target[1]
        sg2 = config["sigma_goal"] ** 2
        S_goal = 0.5 * (gx * gx + gy * gy) / sg2
        # Gradient of S_goal with respect to my position: points AWAY from
        # the target, so moving against it brings me closer.
        grad_x, grad_y = gx / sg2, gy / sg2

        # --- Expectation 2: neighbours at desired_distance.
        S_neigh = 0.0
        push_x, push_y = 0.0, 0.0
        protective = False
        for j, (xj, yj) in enumerate(positions):
            if j == i:
                continue
            dx, dy = xi - xj, yi - yj
            d = max(math.hypot(dx, dy), 1e-6)
            ux, uy = dx / d, dy / d              # unit vector from neighbour j to me
            e = d - config["desired_distance"]   # >0: too far, <0: too close
            sd2 = config["sigma_distance"] ** 2
            S_neigh += 0.5 * e * e / sd2

            if variant == "surprise":
                # Gradient of S_ij with respect to my position: e/sigma^2
                # along u. Too far (e>0) -> moving against it pulls me
                # TOWARDS j; too close (e<0) -> pushes me AWAY from j.
                grad_x += e / sd2 * ux
                grad_y += e / sd2 * uy

            # Safety push (both variants): get away from a neighbour that
            # is dangerously close, whatever the surprise says.
            if d < config["protective_distance"]:
                protective = True
                strength = config["separation_recovery_gain"] * (config["protective_distance"] - d)
                push_x += strength * ux
                push_y += strength * uy

        F = S_goal + (S_neigh if variant == "surprise" else 0.0)

        # Move DOWN the gradient (that lowers F), plus the safety push.
        vx = -config["step_gain"] * grad_x + push_x
        vy = -config["step_gain"] * grad_y + push_y
        vx, vy = limit_speed(vx, vy, config["max_horizontal_speed"])

        out.append({"vx": vx, "vy": vy, "S_goal": S_goal,
                    "S_neighbours": S_neigh, "F": F, "protective": protective})
    return out
