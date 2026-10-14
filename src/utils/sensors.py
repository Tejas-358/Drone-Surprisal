"""
sensors.py — stream each drone's estimated position and velocity to the PC.

The drone's onboard estimator ("stateEstimate") combines its sensors into
a best guess of where it is and how fast it moves. In the simulator this
comes from CrazySim; on real hardware it comes from the Lighthouse
positioning system. Either way the variable names are the same, so the
code below is identical for sim and real.

Used by: sanity tests (test_connection, hover_telemetry_test) and every
experiment, which read the latest x / vx etc. from here in their control loop.
"""

import time

from cflib.crazyflie.log import LogConfig


# The six values we stream. Six floats = 24 bytes, which fits in one
# Crazyflie log packet (max 26 bytes), so they all arrive together.
STATE_VARIABLES = ["x", "y", "z", "vx", "vy", "vz"]


def start_state_logging(scf, period_ms):
    """
    Ask one drone to send its position and velocity every period_ms.

    Input:  scf       — one connected drone (from connect_all)
            period_ms — how often to send, e.g. 100 (= 10 Hz)
    Output: (state, log_config)
            state      — a dictionary that is updated in the background
                         with the latest values: state["x"], state["vx"], ...
                         plus state["received_at"] (PC time of the last packet)
                         and, on newer firmware, state["supervisor"] (status bits,
                         see flight.describe_supervisor()).
                         It stays EMPTY until the first packet arrives.
            log_config — keep this so you can stop the stream later.

    Called from: once per drone, right after connecting.
    """
    log_config = LogConfig(name="State", period_in_ms=period_ms)

    for name in STATE_VARIABLES:
        log_config.add_variable(f"stateEstimate.{name}", "float")

    # The supervisor status (armed? locked? crashed?) — 2 more bytes, so the
    # packet is exactly full (26 bytes). Only newer firmware has it, so we
    # check the drone's list of log variables first.
    has_supervisor = scf.cf.log.toc.get_element_by_complete_name("supervisor.info") is not None
    if has_supervisor:
        log_config.add_variable("supervisor.info", "uint16_t")

    state = {}

    # CFLib calls this in the background every time a packet arrives.
    # We just copy the newest values into the `state` dictionary, so the
    # control loop can read them whenever it likes.
    def callback(timestamp, data, logconf):
        for name in STATE_VARIABLES:
            state[name] = data[f"stateEstimate.{name}"]
        if has_supervisor:
            state["supervisor"] = data["supervisor.info"]
        # PC time when this packet arrived, so safety.py can spot stale data.
        state["received_at"] = time.time()

    log_config.data_received_cb.add_callback(callback)
    scf.cf.log.add_config(log_config)
    log_config.start()

    return state, log_config


def start_all(drones, period_ms):
    """
    Start state logging on every drone.

    Input:  drones    — list from connect_all()
            period_ms — how often each drone sends data
    Output: (states, log_configs) — two lists in the same order as drones,
            so states[0] belongs to CF1, states[1] to CF2, ...

    Called from: experiments and tests, before takeoff.
    """
    states = []
    log_configs = []

    for scf in drones:
        state, log_config = start_state_logging(scf, period_ms)
        states.append(state)
        log_configs.append(log_config)

    return states, log_configs


def wait_for_data(states, timeout=5.0):
    """
    Wait until every drone has sent at least one packet.

    Input:  states  — list from start_all()
            timeout — give up after this many seconds
    Output: none. Raises an error if data never arrives, so the
            experiment stops BEFORE it tries to control on missing data.

    Called from: right after start_all(), before takeoff.
    """
    start_time = time.time()

    while not all(states):   # an empty dictionary counts as False
        if time.time() - start_time > timeout:
            missing = [f"CF{i + 1}" for i, s in enumerate(states) if not s]
            raise RuntimeError(f"No telemetry from {', '.join(missing)}")
        time.sleep(0.1)


def stop_all(log_configs):
    """
    Stop the data streams.

    Input:  log_configs — list from start_all()
    Output: none. Errors are ignored (the link may already be gone).

    Called from: the end of a test or experiment, before disconnecting.
    """
    for log_config in log_configs:
        try:
            log_config.stop()
            # Also remove it from the drone, so a later connection does not
            # get leftover packets ("Error no LogEntry to handle id=...").
            log_config.delete()
        except Exception:
            pass



def read_battery(scf):
    """
    Read one drone's battery voltage once.

    Input:  scf — one connected drone
    Output: battery voltage in volts (float), or None if nothing arrived.
            A full battery is ~4.2 V at rest; below ~3.8 V a Crazyflie with
            a Lighthouse deck may not have the power to take off.

    Called from: safety.preflight_check(), before arming.
    """
    log_config = LogConfig(name="Battery", period_in_ms=100)
    log_config.add_variable("pm.vbat", "float")
    result = {}

    def callback(timestamp, data, logconf):
        result["vbat"] = data["pm.vbat"]

    log_config.data_received_cb.add_callback(callback)
    scf.cf.log.add_config(log_config)
    log_config.start()

    # Wait up to 2 s for the first packet.
    end = time.time() + 2.0
    while "vbat" not in result and time.time() < end:
        time.sleep(0.05)

    stop_all([log_config])
    return result.get("vbat")
