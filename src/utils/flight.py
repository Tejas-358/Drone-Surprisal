"""
flight.py — take off, land, and the safety wrapper that ALWAYS lands.

Every script flies the same way:
    connect -> take off -> do its mission -> land -> disconnect
fly_safely() does all of that, and puts the landing inside a `finally:`
block, so the drones land even if the mission crashes or you press Ctrl-C.

Used by: every sanity test (src/tests/) and every experiment
(src/experiments/). Scripts should call fly_safely() rather than
calling takeoff/land themselves.
"""

import time

from src.utils.connection import connect_all, disconnect_all


def takeoff_all(drones, config):
    """
    Send a takeoff command to every drone and wait until they are hovering.

    Input:  drones — list from connect_all()
            config — dictionary from load_config() (uses height, takeoff_time)
    Output: none.

    Called from: fly_safely(), before the mission starts.
    """
    print(f"\nTaking off to {config['height']} m...")

    for scf in drones:
        # High-level commander: the drone plans its own smooth climb.
        scf.cf.high_level_commander.takeoff(config["height"], config["takeoff_time"])

    # Wait for the climb to finish, plus 1 s to settle.
    time.sleep(config["takeoff_time"] + 1.0)
    print("Hovering.")


def land_all(drones, config):
    """
    Land every drone safely, then switch the motors off.

    Input:  drones — list from connect_all()
            config — dictionary from load_config() (uses land_time)
    Output: none.

    Safe to call at ANY moment, even after an error. Each step is wrapped
    in try/except so one broken drone does not stop the others landing.

    Called from: the `finally:` block in fly_safely().
    """
    print("\nLanding...")

    for scf in drones:
        try:
            # IMPORTANT: if the script has been sending low-level setpoints
            # (e.g. send_velocity_world_setpoint), the firmware ignores
            # high-level commands like land() until we say "I'm done with
            # low-level setpoints". This line says that. Without it, a real
            # drone may keep hovering instead of landing.
            scf.cf.commander.send_notify_setpoint_stop()

            scf.cf.high_level_commander.land(0.0, config["land_time"])
        except Exception as error:
            print(f"Land command failed on one drone: {error}")

    # Wait for the descent to finish, plus 1 s to touch down.
    time.sleep(config["land_time"] + 1.0)

    stop_motors(drones)
    print("Landed.")


def stop_motors(drones):
    """
    Cut the motors on every drone immediately.

    Input:  drones — list from connect_all()
    Output: none.

    Called from: land_all() once the drones are on the ground. It can also
    be used on its own as an emergency stop (the drone will drop!).
    """
    for scf in drones:
        try:
            # A "stop" setpoint = zero thrust, motors off.
            scf.cf.commander.send_stop_setpoint()
        except Exception:
            pass


def fly_safely(config, n_drones, mission):
    """
    Connect, take off, run the mission, then ALWAYS land and disconnect.

    Input:  config   — dictionary from load_config()
            n_drones — how many drones to use (1 or 3)
            mission  — a function you write, taking (drones, config).
                       It runs while the drones are hovering. Whatever it
                       returns is passed back to the caller.
    Output: the mission's return value.

    Example (from a test script):
        def hover(drones, config):
            time.sleep(3)
        fly_safely(config, 1, hover)

    Called from: main() in every test and experiment.
    """
    drones = connect_all(config, n_drones)

    try:
        takeoff_all(drones, config)
        return mission(drones, config)

    finally:
        # This block runs no matter what: normal finish, error, or Ctrl-C.
        land_all(drones, config)
        disconnect_all(drones)
