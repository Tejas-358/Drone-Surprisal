"""
flight.py — take off, land, and the safety wrapper that ALWAYS lands.

Every script flies the same way:
    connect -> take off -> do its mission -> land -> disconnect
fly_safely() does all of that, and puts the landing inside a `finally:`
block, so the drones land even if the mission crashes or you press Ctrl-C.
If safety.py raises EmergencyStop (a drone outside the arena or flying away),
THAT drone's motors are cut immediately; the other drones still land.

Used by: every sanity test (src/tests/) and every experiment
(src/experiments/). Scripts should call fly_safely() rather than
calling takeoff/land themselves.
"""

import time

from src.utils.connection import connect_all, disconnect_all
from src.utils import sensors
from src.utils.safety import preflight_check, watch, EmergencyStop


def set_armed(scf, armed):
    """
    Arm (armed=True) or disarm (armed=False) one drone.

    Recent Crazyflie firmware will not spin the motors until the drone is
    armed. Newer cflib versions use cf.supervisor for this, older ones
    cf.platform — we try the new one first and fall back to the old one.

    Called from: takeoff_all() (arm) and stop_motors() (disarm).
    """
    try:
        scf.cf.supervisor.send_arming_request(armed)
    except AttributeError:
        scf.cf.platform.send_arming_request(armed)


# Meaning of each bit in the firmware's "supervisor.info" status number.
SUPERVISOR_BITS = ["can be armed", "ARMED", "auto-arm", "can fly", "FLYING",
                   "TUMBLED", "LOCKED (power-cycle needed)", "CRASHED"]


def describe_supervisor(state):
    """
    Turn a drone's supervisor status number into readable words.

    Input:  state — one drone's telemetry dictionary (from sensors.py)
    Output: e.g. "can be armed, ARMED, can fly", or a note if unknown.

    Called from: takeoff_all(), to explain why a drone did not arm or fly.
    """
    if "supervisor" not in state:
        return "status unknown (older firmware)"
    on = [name for bit, name in enumerate(SUPERVISOR_BITS) if state["supervisor"] & (1 << bit)]
    return ", ".join(on) if on else "nothing set"


def is_armed(state):
    """True if the drone reports it is armed (or if the firmware can't tell us)."""
    return "supervisor" not in state or bool(state["supervisor"] & (1 << 1))


def takeoff_all(drones, states, config):
    """
    Send a takeoff command to every drone and wait until they are hovering.

    Input:  drones — list from connect_all()
            states — live telemetry (from sensors.start_all), for safety checks
            config — dictionary from load_config() (uses height, takeoff_time)
    Output: none.

    Called from: fly_safely(), before the mission starts.
    """
    print("\nArming...")
    for scf in drones:
        set_armed(scf, True)
    time.sleep(1.0)   # give the firmware a moment to arm

    # Did every drone really arm? A request can get lost on a busy radio,
    # so ask a second time; if a drone still refuses, stop BEFORE takeoff.
    for i, (scf, state) in enumerate(zip(drones, states)):
        if not is_armed(state):
            set_armed(scf, True)
            time.sleep(1.0)
        if not is_armed(state):
            raise RuntimeError(f"CF{i + 1} refused to arm. Status: {describe_supervisor(state)}. "
                               "Check battery; power-cycle it if LOCKED/CRASHED/TUMBLED.")

    print(f"Taking off to {config['height']} m...")

    for scf in drones:
        # The previous flight ended with a low-level "stop" command. Until we
        # say "low-level is finished", the firmware IGNORES high-level
        # commands like takeoff() — the drone stays on the floor although it
        # is armed. (This is why drones used to need a reset between runs.)
        scf.cf.commander.send_notify_setpoint_stop()

        # High-level commander: the drone plans its own smooth climb.
        scf.cf.high_level_commander.takeoff(config["height"], config["takeoff_time"])

    # All drones got the takeoff command at the same moment above, so they
    # climb together.

    # Wait for the climb to finish, plus 1 s to settle — checking safety
    # the whole time, so a drone that flies off is stopped straight away.
    watch(states, config, config["takeoff_time"] + 1.0)

    # Did every drone really leave the ground? If one is still down, stop
    # here (the others get landed) instead of flying an incomplete group.
    for i, state in enumerate(states):
        if state["z"] < 0.5 * config["height"]:
            raise RuntimeError(f"CF{i + 1} did not take off (z={state['z']:.2f} m). "
                               f"Status: {describe_supervisor(state)}.")

    print("Hovering.")


def land_all(drones, states, config):
    """
    Land every drone safely, then switch the motors off.

    Input:  drones — list of drones to land
            states — their live telemetry (same order), to see when they touch down
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

    # Wait until every drone is really on the floor (z below 10 cm) before
    # cutting its motors — cutting earlier would drop it. Give up after
    # land_time + 3 s (e.g. if a drone's data stopped arriving).
    end = time.time() + config["land_time"] + 3.0
    while time.time() < end:
        if all(s.get("z", 0.0) < 0.1 for s in states):
            break
        time.sleep(0.1)
    time.sleep(0.5)   # let it settle on the floor

    stop_motors(drones)
    print("Landed.")


def fly_to_positions(drones, states, config, positions):
    """
    Move hovering drones to start positions WITHOUT their paths colliding.

    In the simulator drones always spawn at fixed spots, and on real drones
    hand placement is never exact, so experiments fly to their start
    positions first. Moving everyone at the same height could make paths
    cross, so each drone travels at its OWN height:
        1. drone i climbs to  height + 0.25 m * i
        2. all drones fly sideways to their (x, y)  — at different heights
        3. all drones descend to the flight height

    Input:  drones    — hovering drones (from fly_safely)
            states    — live telemetry, for safety checks while moving
            config    — uses height, transfer_time (s, optional, default 4)
            positions — list of (x, y) in metres, one per drone, in order
    Output: none.

    Called from: experiments, right after takeoff.
    """
    height = config["height"]
    travel = config.get("transfer_time", 4.0)
    layers = [height + 0.25 * i for i in range(len(drones))]

    print("\nFlying to start positions (each drone at its own height)...")
    for scf, s, z in zip(drones, states, layers):              # 1. climb in place
        scf.cf.high_level_commander.go_to(s["x"], s["y"], z, 0.0, 2.0)
    watch(states, config, 2.5)

    for scf, (x, y), z in zip(drones, positions, layers):      # 2. move sideways
        scf.cf.high_level_commander.go_to(x, y, z, 0.0, travel)
    watch(states, config, travel + 0.5)

    for scf, (x, y) in zip(drones, positions):                 # 3. descend
        scf.cf.high_level_commander.go_to(x, y, height, 0.0, 2.0)
    watch(states, config, 3.0)


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
            # Disarm, so the motors cannot start again by accident.
            set_armed(scf, False)
        except Exception:
            pass


def fly_safely(config, n_drones, mission):
    """
    Connect, take off, run the mission, then ALWAYS land and disconnect.

    Input:  config   — dictionary from load_config()
            n_drones — how many drones to use (1 or 3)
            mission  — a function you write, taking (drones, config, states).
                       It runs while the drones are hovering. `states` is
                       the live telemetry (states[0]["x"] = CF1's x, ...).
                       Whatever it returns is passed back to the caller.
    Output: the mission's return value.

    Example (from a test script):
        def hover(drones, config, states):
            watch(states, config, 3)   # like sleep(3), but with safety checks
        fly_safely(config, 1, hover)

    Called from: main() in every test and experiment.
    """
    drones = connect_all(config, n_drones)
    states = []
    log_configs = []
    took_off = False     # becomes True once takeoff has been commanded
    cut = None           # index of a drone whose motors were cut in an emergency

    try:
        # One telemetry stream per drone for the whole flight.
        states, log_configs = sensors.start_all(drones, config["log_period_ms"])
        sensors.wait_for_data(states)

        # Check positions BEFORE arming. If this fails, nothing takes off.
        preflight_check(drones, states, config)

        took_off = True
        takeoff_all(drones, states, config)
        return mission(drones, config, states)

    except EmergencyStop as error:
        # This drone's position looks broken: do NOT try to fly it anywhere.
        # Cut ONLY its motors; the other drones are landed normally below.
        print()
        print(error)
        print(f"Cutting motors of CF{error.drone + 1}!")
        cut = error.drone
        stop_motors([drones[cut]])
        raise

    finally:
        # Runs no matter what: normal finish, error, or Ctrl-C.
        if took_off:
            keep = [i for i in range(len(drones)) if i != cut]
            land_all([drones[i] for i in keep], [states[i] for i in keep], config)
        stop_motors(drones)   # motors off + disarm (harmless if already off)
        sensors.stop_all(log_configs)
        disconnect_all(drones)

