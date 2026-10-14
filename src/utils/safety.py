"""
safety.py — checks that stop a flight BEFORE something goes wrong.

Two checks:
  preflight_check() — before arming: is every drone's position valid,
                      on the floor, inside the arena, and (for experiments)
                      sitting on its start spot?
  check_in_flight() — every control step: is every drone inside the
                      arena, is its data fresh, and are drones too close?

Both simply RAISE an error when something is wrong. fly_safely() in
flight.py catches every error by landing all drones, so "raise" here
means "abort and land".

Used by: flight.py (preflight, for every test and experiment) and the
experiment control loops (in-flight, every step).
"""

import time

from src.utils import sensors


def preflight_check(drones, config):
    """
    Refuse to take off unless every drone has a sensible position.

    Input:  drones — list from connect_all()
            config — uses mode, arena, and start_x if the experiment has one
    Output: none. Raises RuntimeError (nothing takes off) if a check fails.

    Called from: fly_safely(), after connecting and BEFORE arming.
    """
    # 1. On real drones: is the Lighthouse deck detected?
    #    (CrazySim drones have no physical decks, so skip this in sim.)
    if config["mode"] == "real":
        for i, scf in enumerate(drones):
            if int(scf.cf.param.get_value("deck.bcLighthouse4")) != 1:
                raise RuntimeError(f"CF{i + 1}: Lighthouse deck not detected. Not flying.")

    # 2. Read every drone's current position for a moment.
    states, log_configs = sensors.start_all(drones, config["log_period_ms"])
    try:
        sensors.wait_for_data(states)
        time.sleep(0.5)   # let a few fresh packets arrive
        positions = [(s["x"], s["y"], s["z"]) for s in states]
    finally:
        sensors.stop_all(log_configs)

    arena = config["arena"][config["mode"]]   # separate limits for sim and real
    for i, (x, y, z) in enumerate(positions):
        name = f"CF{i + 1}"
        print(f"{name} on the ground at x={x:.2f} y={y:.2f} z={z:.2f}")

        # On the floor? A position like z = 108 means "no real position fix".
        if abs(z) > 0.15:
            raise RuntimeError(f"{name}: z={z:.2f} m but it should be on the floor. "
                               "Position estimate is not valid. Not flying.")

        # Inside the arena?
        if not (arena["x_min"] <= x <= arena["x_max"] and
                arena["y_min"] <= y <= arena["y_max"]):
            raise RuntimeError(f"{name}: x={x:.2f} y={y:.2f} is outside the arena. Not flying.")

        # Real experiments: is the drone placed on its start spot? Wrong
        # placement would make the drones' paths cross when they fly to the
        # start line. (In sim, CrazySim decides where drones spawn.)
        if config["mode"] == "real" and "start_x" in config:
            start = config["start_x"][i]
            if abs(x - start) > 0.3 or abs(y) > 0.3:
                raise RuntimeError(f"{name}: place it at x={start}, y=0 "
                                   f"(it is at x={x:.2f}, y={y:.2f}). Not flying.")

    print("Pre-flight check passed.")


def check_in_flight(states, config, min_gap=None):
    """
    Abort the flight if anything looks unsafe. Call once per control step.

    Input:  states  — list from sensors.start_all() (latest data per drone)
            config  — uses arena and max_data_age
            min_gap — optional: abort if neighbouring drones (in list order,
                      drone i+1 behind drone i along x) get closer than this (m)
    Output: none. Raises RuntimeError -> fly_safely() lands every drone.

    Called from: the control loops in the experiment scripts.
    """
    arena = config["arena"][config["mode"]]
    now = time.time()

    for i, s in enumerate(states):
        name = f"CF{i + 1}"

        # Old data means the radio link is dropping packets. Acting on an
        # old position is dangerous, so stop.
        age = now - s["received_at"]
        if age > config["max_data_age"]:
            raise RuntimeError(f"ABORT: no fresh data from {name} for {age:.2f} s")

        if not (arena["x_min"] <= s["x"] <= arena["x_max"] and
                arena["y_min"] <= s["y"] <= arena["y_max"] and
                s["z"] <= arena["z_max"]):
            raise RuntimeError(f"ABORT: {name} left the arena "
                               f"(x={s['x']:.2f} y={s['y']:.2f} z={s['z']:.2f})")

    if min_gap is not None:
        for i in range(len(states) - 1):
            gap = states[i]["x"] - states[i + 1]["x"]
            if gap < min_gap:
                raise RuntimeError(f"ABORT: CF{i + 1} and CF{i + 2} only {gap:.2f} m apart")
