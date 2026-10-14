"""
safety.py — checks that stop a flight BEFORE something goes wrong.

  preflight_check() — before arming: is every drone's position valid,
                      STEADY, on the floor, inside the arena, and (real
                      experiments) sitting on its start spot?
  check_in_flight() — while flying: is every drone inside the arena, not
                      flying too fast, is its data fresh, are drones too close?
  watch()           — "sleep" for a while, running check_in_flight() 10x
                      per second. Use it instead of time.sleep() in flight.

Two kinds of error:
  RuntimeError   -> fly_safely() LANDS all drones (e.g. stale data, drones
                    too close: the position is still trustworthy).
  EmergencyStop  -> fly_safely() CUTS THE MOTORS at once (drone outside the
                    arena or flying too fast: the position estimate is
                    probably broken, so trying to "land" could fly it into
                    a wall. Dropping from < 1 m is safer.)

Used by: flight.py (preflight + watch during takeoff/hover) and the
experiment control loops (check_in_flight every step).
"""

import time

from src.utils import sensors


class EmergencyStop(RuntimeError):
    """Raised when the motors must be cut immediately (see top of file)."""


def preflight_check(drones, states, config):
    """
    Refuse to take off unless every drone has a valid, steady position.

    Input:  drones — list from connect_all()
            states — list from sensors.start_all() (already streaming)
            config — uses mode, arena, preflight_max_spread, and start_x if
                     the experiment has one
    Output: none. Raises RuntimeError (nothing takes off) if a check fails.

    Called from: fly_safely(), after connecting and BEFORE arming.
    """
    # 1. On real drones: is the Lighthouse deck detected, and is the
    #    battery charged enough to lift off?
    #    (CrazySim drones have no physical decks or batteries, so skip in sim.)
    if config["mode"] == "real":
        for i, scf in enumerate(drones):
            if int(scf.cf.param.get_value("deck.bcLighthouse4")) != 1:
                raise RuntimeError(f"CF{i + 1}: Lighthouse deck not detected. Not flying.")

            vbat = sensors.read_battery(scf)
            print(f"CF{i + 1} battery: {vbat:.2f} V" if vbat else f"CF{i + 1} battery: unknown")
            if vbat is not None and vbat < config["min_battery_v"]:
                raise RuntimeError(f"CF{i + 1}: battery {vbat:.2f} V is below "
                                   f"{config['min_battery_v']} V. Charge it. Not flying.")


    # 2. Record each drone's position 10 times over 1 second.
    samples = [[] for _ in states]
    for _ in range(10):
        for i, s in enumerate(states):
            samples[i].append((s["x"], s["y"], s["z"]))
        time.sleep(0.1)

    arena = config["arena"][config["mode"]]   # separate limits for sim and real

    for i, points in enumerate(samples):
        name = f"CF{i + 1}"
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        x, y, z = xs[-1], ys[-1], zs[-1]

        # Steady? The drone is sitting still, so the estimate should barely
        # move. If it jumps around, Lighthouse is not tracking it properly
        # and the drone would chase those jumps in flight.
        spread = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        print(f"{name} on the ground at x={x:.2f} y={y:.2f} z={z:.2f} "
              f"(jitter {spread * 100:.1f} cm)")
        if spread > config["preflight_max_spread"]:
            raise RuntimeError(
                f"{name}: position jumps by {spread * 100:.0f} cm while sitting "
                "still. Lighthouse tracking is not reliable. Not flying.")

        # On the floor? z far from 0 means the estimate is wrong.
        if abs(z) > 0.1:
            raise RuntimeError(f"{name}: z={z:.2f} m but it is on the floor. "
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
    Stop the flight if anything looks unsafe. Call once per control step.

    Input:  states  — list from sensors.start_all() (latest data per drone)
            config  — uses arena, max_speed, max_data_age
            min_gap — optional: land if neighbouring drones (in list order,
                      drone i+1 behind drone i along x) get closer than this (m)
    Output: none.
            Raises EmergencyStop (-> motors cut) if a drone is outside the
            arena or too fast; RuntimeError (-> land) for the other problems.

    Called from: watch() below and the experiment control loops.
    """
    arena = config["arena"][config["mode"]]
    now = time.time()

    for i, s in enumerate(states):
        name = f"CF{i + 1}"

        if not (arena["x_min"] <= s["x"] <= arena["x_max"] and
                arena["y_min"] <= s["y"] <= arena["y_max"] and
                s["z"] <= arena["z_max"]):
            raise EmergencyStop(f"EMERGENCY STOP: {name} outside the arena "
                                f"(x={s['x']:.2f} y={s['y']:.2f} z={s['z']:.2f})")

        # Much faster than any of our manoeuvres -> the drone is flying away.
        speed = (s["vx"] ** 2 + s["vy"] ** 2 + s["vz"] ** 2) ** 0.5
        if speed > config["max_speed"]:
            raise EmergencyStop(f"EMERGENCY STOP: {name} moving at {speed:.2f} m/s")

        # Old data means the radio link is dropping packets. Acting on an
        # old position is dangerous, so land.
        age = now - s["received_at"]
        if age > config["max_data_age"]:
            raise RuntimeError(f"ABORT: no fresh data from {name} for {age:.2f} s")

    if min_gap is not None:
        for i in range(len(states) - 1):
            gap = states[i]["x"] - states[i + 1]["x"]
            if gap < min_gap:
                raise RuntimeError(f"ABORT: CF{i + 1} and CF{i + 2} only {gap:.2f} m apart")


def watch(states, config, seconds):
    """
    Wait for `seconds`, checking safety 10 times per second.

    Input:  states  — list from sensors.start_all()
            config  — passed on to check_in_flight()
            seconds — how long to wait
    Output: none. Raises (see check_in_flight) as soon as something is unsafe.

    Called from: flight.py during takeoff, and the hover tests.
    """
    end = time.time() + seconds
    while time.time() < end:
        check_in_flight(states, config)
        time.sleep(0.1)
