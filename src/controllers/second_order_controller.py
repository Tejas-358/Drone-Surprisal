"""
second_order_controller.py — SECOND-ORDER (acceleration) control laws for
the two-drone pair (EXPERIMENTS.md §7) and the three-drone swarm (§8).

Earlier controllers set a VELOCITY from the error (first order). These set
an ACCELERATION — a spring plus a shock absorber:

    a   = -kp * e - kd * rate          spring - damper
    kd  = 2 * sqrt(kp)                 critical damping: no overshoot, fastest settle

The Crazyflie only takes velocity setpoints, so the experiment scripts
integrate:  v_cmd <- clamp(v_cmd + a * dt).  From rest this gives a
bell-shaped speed profile (fast far from the target, slow near it), and
the time to reach the target follows Fitts' law while the speed limit is
not reached.

Measured rates are noisy (Lighthouse jitter) and damping multiplies them,
so every rate is low-pass filtered first (RateFilter).

This file is pure maths (positions in, accelerations out): it never talks to
a drone, so it runs unchanged in the simulator and on real drones.

Used by: src/experiments/second_order_pair_experiment.py
         src/experiments/second_order_swarm_experiment.py
"""

import math


def clamp(value, low, high):
    """Keep value inside [low, high]."""
    return max(low, min(high, value))


def limit_speed(vx, vy, max_speed):
    """Shrink (vx, vy) to at most max_speed, keeping its direction."""
    speed = math.hypot(vx, vy)
    if speed > max_speed:
        return vx * max_speed / speed, vy * max_speed / speed
    return vx, vy


class RateFilter:
    """
    First-order low-pass filter:  r <- (1 - alpha) * r + alpha * measured.
    alpha = 1 means no filtering; smaller alpha = smoother but later.
    """

    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, measured):
        if self.value is None:
            self.value = measured
        else:
            self.value = (1.0 - self.alpha) * self.value + self.alpha * measured
        return self.value


# ============================================================
# Two drones (§7)
# ============================================================

def pair_gains(surprisal, variant, config):
    """
    Spring stiffness kp and critically matched damping kd for this step.

    second_order:          kp = base_kp                                (constant)
    second_order_adaptive: kp = min(base_kp * (1 + surprise_gain * S), max_kp)
    Both:                  kd = 2 * sqrt(kp)   (recomputed every step, so the
                           brakes always match the spring)
    """
    if variant == "second_order":
        kp = config["base_kp"]
    elif variant == "second_order_adaptive":
        kp = min(config["base_kp"] * (1.0 + config["surprise_gain"] * surprisal),
                 config["max_kp"])
    else:
        raise ValueError(f"Unknown variant: {variant}")
    return kp, 2.0 * math.sqrt(kp)


def pair_step(d, rate, u, variant, config):
    """
    One control step for the gap between two drones.

    Input:  d       — measured distance between the drones (m)
            rate    — FILTERED rate of change of d (m/s, >0: drifting apart)
            u       — current gap-rate command (m/s), from the previous step
            variant — "second_order" or "second_order_adaptive"
            config  — uses desired_distance, sigma_distance, base_kp,
                      surprise_gain, max_kp, max_gap_rate, control_period,
                      protective_distance, separation_recovery_gain
    Output: dictionary with
            "e", "S", "kp", "kd", "accel"  — intermediate values (for logging)
            "u"           — new gap-rate command (m/s): >0 opens, <0 closes
            "protective"  — True if the push-apart overrode the spring
    """
    e = d - config["desired_distance"]                 # >0: too far apart
    S = 0.5 * (e / config["sigma_distance"]) ** 2
    kp, kd = pair_gains(S, variant, config)

    accel = -kp * e - kd * rate                        # wanted gap acceleration
    u = clamp(u + accel * config["control_period"],
              -config["max_gap_rate"], config["max_gap_rate"])

    protective = d < config["protective_distance"]
    if protective:
        # Too close: open the gap at least this fast. Replacing u (not just
        # adding) also clears the integrator, so it cannot wind up.
        u = max(u, config["separation_recovery_gain"] * (config["protective_distance"] - d))

    return {"e": e, "S": S, "kp": kp, "kd": kd, "accel": accel,
            "u": u, "protective": protective}


# ============================================================
# Three drones (§8)
# ============================================================

def swarm_damping(config):
    """
    Critical damping for the two kinds of spring in the swarm.

    goal spring:      stiffness kappa / sigma_goal^2 on one drone
                      -> omega_g = sqrt(kappa) / sigma_goal,  c_g = 2 * omega_g
    neighbour spring: stiffness kappa / sigma_distance^2, but BOTH drones of a
                      pair move, so the pair distance sees twice that
                      -> omega_n = sqrt(2 kappa) / sigma_distance,  c_n = omega_n
                      (each drone brakes with c_n, the pair with 2 c_n = 2 omega_n)
    Output: (c_g, c_n, omega_g, omega_n)
    """
    kappa = config["kappa"]
    omega_g = math.sqrt(kappa) / config["sigma_goal"]
    omega_n = math.sqrt(2.0 * kappa) / config["sigma_distance"]
    return 2.0 * omega_g, omega_n, omega_g, omega_n


def swarm_accelerations(positions, velocities, pair_rates, target, config, variant):
    """
    Acceleration for every drone: accelerate DOWN the same surprise F as the
    first-order target swarm, with one damper per spring.

        a_i = -kappa * grad F_i  -  c_g * v_i  -  sum_j c_n * r_ij * u_ij

    Input:  positions  — list of (x, y) per drone (m), what the drones see
            velocities — list of (vx, vy) per drone (m/s), what the drones see
            pair_rates — dict {(i, j): filtered rate of d_ij} for i < j (m/s)
            target     — (x, y) of the target
            config     — uses kappa, sigma_goal, sigma_distance, desired_distance
            variant    — "second_order" (target + neighbours) or
                         "goal_only_second_order" (target only, baseline)
    Output: list of dictionaries, one per drone, with
            "ax", "ay"      — acceleration command (m/s^2)
            "S_goal"        — surprise about the target
            "S_neighbours"  — sum of surprises about the other drones (logged
                              in BOTH variants)
            "F"             — total surprise this drone acts on
    """
    kappa = config["kappa"]
    sg2 = config["sigma_goal"] ** 2
    sd2 = config["sigma_distance"] ** 2
    c_g, c_n, _, _ = swarm_damping(config)

    out = []
    for i, ((xi, yi), (vxi, vyi)) in enumerate(zip(positions, velocities)):
        # --- Goal spring + damper on my own velocity.
        gx, gy = xi - target[0], yi - target[1]
        S_goal = 0.5 * (gx * gx + gy * gy) / sg2
        ax = -kappa * gx / sg2 - c_g * vxi
        ay = -kappa * gy / sg2 - c_g * vyi

        # --- Neighbour springs + dampers on how fast each distance changes.
        S_neigh = 0.0
        for j, (xj, yj) in enumerate(positions):
            if j == i:
                continue
            dx, dy = xi - xj, yi - yj
            d = max(math.hypot(dx, dy), 1e-6)
            ux, uy = dx / d, dy / d                  # unit vector from j to me
            e = d - config["desired_distance"]       # >0: too far
            S_neigh += 0.5 * e * e / sd2
            if variant == "second_order":
                rate = pair_rates[(min(i, j), max(i, j))]
                # Too far or separating -> accelerate towards j (along -u).
                strength = kappa * e / sd2 + c_n * rate
                ax -= strength * ux
                ay -= strength * uy

        F = S_goal + (S_neigh if variant == "second_order" else 0.0)
        out.append({"ax": ax, "ay": ay, "S_goal": S_goal,
                    "S_neighbours": S_neigh, "F": F})
    return out


def safety_push(positions, config):
    """
    Push-apart velocity per drone for any neighbour closer than
    protective_distance (both variants; not part of the surprise).
    Output: list of (push_x, push_y, active) per drone.
    """
    pushes = []
    for i, (xi, yi) in enumerate(positions):
        px, py, active = 0.0, 0.0, False
        for j, (xj, yj) in enumerate(positions):
            if j == i:
                continue
            dx, dy = xi - xj, yi - yj
            d = max(math.hypot(dx, dy), 1e-6)
            if d < config["protective_distance"]:
                active = True
                strength = config["separation_recovery_gain"] * (config["protective_distance"] - d)
                px += strength * dx / d
                py += strength * dy / d
        pushes.append((px, py, active))
    return pushes
