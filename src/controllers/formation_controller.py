"""
formation_controller.py — the control law for E3, the formation-distance
experiment.

Three drones sit in a line along x: CF1 -- CF2 -- CF3. We want each gap
between neighbours to equal desired_distance. We define a "potential"
(think: an energy that is 0 when the formation is perfect):

    S12 = (x2 - x1) - desired_distance     # error in gap CF1-CF2
    S23 = (x3 - x2) - desired_distance     # error in gap CF2-CF3
    U   = 0.5 * (S12^2 + S23^2)

Each step, every drone moves a little bit "downhill" on U (gradient
descent), which shrinks the errors until the gaps are right.

This file only does MATHS: positions in, target positions out. It never
talks to a drone, so it runs unchanged in the simulator and on hardware.

Used by: src/experiments/formation_distance_experiment.py, once per step.
"""


def clamp(value, minimum, maximum):
    """
    Keep a value inside [minimum, maximum].

    Input:  value, minimum, maximum — numbers
    Output: value if it is inside the range, otherwise the nearest limit.

    Called from: formation_step(), to limit how far a drone moves per step.
    """
    return max(minimum, min(maximum, value))


def formation_step(x1, x2, x3, config):
    """
    Compute one gradient-descent step for all three drones.

    Input:  x1, x2, x3 — current x positions of CF1, CF2, CF3 (m),
                         assumed to be in that order along x
            config     — uses desired_distance, gain, max_step
    Output: dictionary with
            "S12", "S23" — gap errors (m); positive = gap too big
            "U"          — formation potential (m^2); 0 = perfect
            "step1", "step2", "step3" — how far each drone moves (m)
            "target1", "target2", "target3" — new x for each drone (m)

    Called from: the control loop in formation_distance_experiment.py.
    """
    d = config["desired_distance"]
    gain = config["gain"]

    # Gap errors. Positive means the gap is too big, negative too small.
    S12 = (x2 - x1) - d
    S23 = (x3 - x2) - d

    U = 0.5 * (S12 ** 2 + S23 ** 2)

    # Gradient descent: each drone moves by -gain * (slope of U for that drone).
    #   CF1 only affects gap 12. If S12 > 0 (gap too big), CF1 moves +x,
    #   towards CF2.                              -> step1 = +gain * S12
    #   CF2 affects both gaps. Too-big gap 12 pulls it back (-x), too-big
    #   gap 23 pulls it forward (+x).             -> step2 = gain * (S23 - S12)
    #   CF3 only affects gap 23. If S23 > 0, CF3 moves -x, towards CF2.
    #                                             -> step3 = -gain * S23
    step1 = gain * S12
    step2 = gain * (S23 - S12)
    step3 = -gain * S23

    # Limit each move so the drones never jump far in a single step.
    m = config["max_step"]
    step1 = clamp(step1, -m, m)
    step2 = clamp(step2, -m, m)
    step3 = clamp(step3, -m, m)

    return {
        "S12": S12,
        "S23": S23,
        "U": U,
        "step1": step1,
        "step2": step2,
        "step3": step3,
        "target1": x1 + step1,
        "target2": x2 + step2,
        "target3": x3 + step3,
    }
