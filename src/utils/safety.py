"""
safety.py — the few safety checks we keep, kept simple.

  preflight_check() — before arming: on real drones, is the Lighthouse deck
                      there? Prints each drone's battery and position, and
                      refuses to fly if a position is clearly NOT REAL
                      (no Lighthouse fix — see below).
  check_in_flight() — while flying: is fresh data still arriving from every
                      drone (radio link OK)? Is every position still real
                      (z not below the floor)? Optionally: drones too close?
  watch()           — "sleep" for a while, running check_in_flight() 10x
                      per second. Use it instead of time.sleep() in flight.

A failed check RAISES an error. fly_safely() in flight.py answers every
error by landing all drones, so "raise" here means "stop and land".

"Is the position real?" — the two checks that matter most. A drone flying
on a broken position estimate goes anywhere (28 Oct: z = -3 m, 35 m/s jumps,
two drones crashed). Signs of no Lighthouse fix:
  - on the floor, z is far from 0 (|z| > 0.1 m);
    (NOT "x, y exactly 0, 0": the Lighthouse setup wizard puts the origin
    where a drone stands, so (0, 0) can be a perfectly real position);
  - in flight, z below the floor (z < -0.1 m).

Deliberately NOT checked (they caused false alarms from small Lighthouse
jumps): speed limit, arena box, position jitter, start spots.
Keep the cfclient Emergency stop button within reach instead.

Used by: flight.py and the experiment control loops.
"""

import time

from src.utils import sensors


def preflight_check(drones, states, config):
    """
    Check the deck and print battery + position of every drone.

    Input:  drones — list from connect_all()
            states — list from sensors.start_all() (already streaming)
            config — uses mode, min_battery_v
    Output: none. Raises RuntimeError only if a real drone has no
            Lighthouse deck (it could not fly at all).

    Called from: fly_safely(), after connecting and BEFORE arming.
    """
    for i, (scf, s) in enumerate(zip(drones, states)):
        name = f"CF{i + 1}"

        # The simulator has no physical decks or batteries.
        if config["mode"] == "real":
            if int(scf.cf.param.get_value("deck.bcLighthouse4")) != 1:
                raise RuntimeError(f"{name}: Lighthouse deck not detected. Not flying.")

            vbat = sensors.read_battery(scf)
            if vbat is not None:
                low = "  <-- LOW, charge it" if vbat < config["min_battery_v"] else ""
                print(f"{name} battery: {vbat:.2f} V{low}")

        print(f"{name} at x={s['x']:.2f} y={s['y']:.2f} z={s['z']:.2f}")

        # Is the position real? A drone on the floor must read z close to 0.
        if abs(s["z"]) > 0.1:
            raise RuntimeError(f"{name}: z={s['z']:.2f} m while on the floor — no "
                               "Lighthouse fix. Run lighthouse_diagnostic. Not flying.")

    print("Pre-flight check passed.")


def check_in_flight(states, config, min_gap=None):
    """
    Stop the flight (land everything) if a drone's data stops arriving, or,
    if min_gap is given, if two neighbouring drones get too close.

    Input:  states  — list from sensors.start_all() (latest data per drone)
            config  — uses max_data_age
            min_gap — optional: drones are in a line along x (drone i+1
                      behind drone i); land if any gap is smaller than this (m)
    Output: none. Raises RuntimeError -> fly_safely() lands every drone.

    Called from: watch() below and the experiment control loops.
    """
    now = time.time()
    for i, s in enumerate(states):
        # Old data means the radio link is dropping packets: we no longer
        # know where this drone is, so land everything.
        age = now - s["received_at"]
        if age > config["max_data_age"]:
            raise RuntimeError(f"ABORT: no fresh data from CF{i + 1} for {age:.2f} s")

        # Below the floor = the position estimate is broken (Lighthouse lost).
        if s["z"] < -0.1:
            raise RuntimeError(f"ABORT: CF{i + 1} reports z={s['z']:.2f} m (below the "
                               "floor) — Lighthouse tracking lost")

    if min_gap is not None:
        for i in range(len(states) - 1):
            gap = states[i]["x"] - states[i + 1]["x"]
            if gap < min_gap:
                raise RuntimeError(f"ABORT: CF{i + 1} and CF{i + 2} only {gap:.2f} m apart")


def watch(states, config, seconds):
    """
    Wait for `seconds`, checking the radio link 10 times per second.

    Called from: flight.py during takeoff / moves, the tests and experiments.
    """
    end = time.time() + seconds
    while time.time() < end:
        check_in_flight(states, config)
        time.sleep(0.1)
