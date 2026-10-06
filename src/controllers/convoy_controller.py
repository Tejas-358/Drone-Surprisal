"""
convoy_controller.py — the control laws for the leader–follower convoy.

This file only does MATHS: it takes positions/velocities in and gives
velocity commands out. It never talks to a drone, never sleeps, and never
knows whether it is running in the simulator or on real hardware.
That is what lets the same controller run unchanged on real Crazyflies.

Used by:
  E1  src/experiments/surprise_minimization_experiment.py  (use_surprise=True)
  E2  src/experiments/convoy_baseline_experiment.py        (use_surprise=False)
Both call leader_command() for CF1 and follower_command() for CF2 and CF3
once per control step. The ONLY difference between E1 and E2 is the
use_surprise flag.

All positions and velocities are along the x axis (the convoy's direction).
"""

import math


def clamp(value, minimum, maximum):
    """
    Keep a value inside [minimum, maximum].

    Input:  value, minimum, maximum — numbers
    Output: value if it is inside the range, otherwise the nearest limit.

    Called from: follower_command(), to apply the speed limit.
    """
    return max(minimum, min(maximum, value))


def leader_command(x_leader, already_stopped, config):
    """
    Decide CF1's (the leader's) forward speed for this step.

    The leader flies forward at a constant speed until it gets within
    wall_stop_distance of the virtual wall, then stops for good.
    The wall is just a number in the config — there is no real obstacle
    and no range sensor; it is only the trigger that stops the leader.

    Input:  x_leader        — CF1's current x position (m)
            already_stopped — True if the leader stopped in an earlier step
            config          — uses wall_x, wall_stop_distance, leader_speed
    Output: (vx, stopped)
            vx      — forward velocity command for CF1 (m/s)
            stopped — True once the leader has stopped (pass it back in
                      as already_stopped on the next step)

    Called from: the control loop of E1 and E2, once per step.
    """
    wall_distance = config["wall_x"] - x_leader

    # Once stopped, stay stopped (even if the drone drifts back a little).
    if already_stopped or wall_distance <= config["wall_stop_distance"]:
        return 0.0, True

    return config["leader_speed"], False


def follower_command(x_ahead, x_self, v_ahead, v_self, config, use_surprise):
    """
    Decide one follower's forward speed for this step.

    Step 1 — normal following (both E1 and E2):
        The follower wants to sit follow_distance behind the drone ahead.
        Its speed command is proportional to how far it is from that spot
        (a "P-controller"), limited to max_follow_speed.

    Step 2 — surprise gating (E1 only, when use_surprise is True):
        The follower PREDICTS that it should be moving at the same speed
        as the drone ahead. The prediction error is
            S = v_ahead - v_self                         (m/s)
        and the surprise is
            U = 0.5 * (S / N)^2                          (no units)
        where N (noise_scale) says how big a speed difference counts as
        "normal". The speed command is then multiplied by
            factor = exp(-surprise_gain * U)
        which is 1 when there is no surprise (move normally) and drops
        towards 0 when surprise is large (slow down / hold still).

    Step 3 — safety repulsion (both E1 and E2):
        If the gap to the drone ahead is smaller than safe_distance, the
        follower backs away at max_follow_speed, overriding steps 1 and 2.

    Input:  x_ahead, x_self — x positions of the drone ahead and this drone (m)
            v_ahead, v_self — x velocities of the drone ahead and this drone (m/s)
            config          — uses follow_distance, follow_gain,
                              max_follow_speed, noise_scale, surprise_gain,
                              safe_distance
            use_surprise    — True for E1, False for E2
    Output: dictionary with every intermediate value (so it can be logged):
            "spacing_error"  — desired x minus actual x (m)
            "base_command"   — P-control speed before gating (m/s)
            "prediction_error" S, "surprise" U, "speed_factor",
            "repelling"      — True if the safety override was used this step
            "command"        — the final speed to send (m/s)
            S, U and speed_factor are computed for E2 as well, purely so
            both experiments log the same columns; E2 does not use them.

    Called from: the control loop of E1 and E2, once per follower per step.
    """
    # --- Step 1: P-control towards the target spot behind the drone ahead
    desired_x = x_ahead - config["follow_distance"]
    spacing_error = desired_x - x_self          # >0 means "too far back, speed up"

    base_command = config["follow_gain"] * spacing_error
    base_command = clamp(base_command,
                         -config["max_follow_speed"],
                         config["max_follow_speed"])

    # --- Step 2: surprise
    prediction_error = v_ahead - v_self         # S: predicted speed minus actual speed
    surprise = 0.5 * (prediction_error / config["noise_scale"]) ** 2   # U

    # exp() of a large negative number just gives 0.0 (no error), so a
    # huge surprise safely means "don't move".
    speed_factor = math.exp(-config["surprise_gain"] * surprise)

    if use_surprise:
        command = base_command * speed_factor   # E1: gated
    else:
        command = base_command                  # E2: plain P-control

    # --- Step 3: safety repulsion (both E1 and E2)
    # If we are dangerously close to the drone ahead, ignore everything
    # above and back away at full speed. This comes LAST so that surprise
    # gating can never cancel it.
    gap = x_ahead - x_self
    repelling = gap < config["safe_distance"]
    if repelling:
        command = -config["max_follow_speed"]   # negative = move backwards

    return {
        "spacing_error": spacing_error,
        "base_command": base_command,
        "prediction_error": prediction_error,
        "surprise": surprise,
        "speed_factor": speed_factor,
        "repelling": repelling,
        "command": command,
    }
