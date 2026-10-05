"""
connection.py — load the config and connect to / disconnect from the drones.

This is the ONLY place in the repo that knows whether we are flying in
the simulator or on real hardware. Everything else just asks this file
for "the drones" and never checks the mode itself.

Used by: every sanity test (src/tests/) and every experiment
(src/experiments/) at the very start (connect) and very end (disconnect).
"""

import yaml

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


DEFAULT_CONFIG = "config/default.yaml"


def load_config(experiment_config=None):
    """
    Read config/default.yaml and, optionally, one experiment's config file.

    Input:  experiment_config — path to e.g. config/experiments/convoy.yaml,
            or None if the script only needs the shared defaults.
    Output: one dictionary with all settings. If a key appears in both
            files, the experiment file's value wins.

    Called from: the top of main() in every test and experiment script.
    """
    with open(DEFAULT_CONFIG) as f:
        config = yaml.safe_load(f)

    if experiment_config is not None:
        with open(experiment_config) as f:
            # update() copies every key from the experiment file over the defaults
            config.update(yaml.safe_load(f))

    return config


def get_uris(config, n_drones):
    """
    Pick the first n_drones addresses for the current mode (sim or real).

    Input:  config   — dictionary from load_config()
            n_drones — how many drones this script needs (1 or 3)
    Output: list of URI strings, e.g. ["udp://127.0.0.1:19850"]

    Called from: connect_all() below.
    """
    mode = config["mode"]
    uris = config["uris"][mode]

    if n_drones > len(uris):
        raise ValueError(
            f"Need {n_drones} drones but config only lists "
            f"{len(uris)} URIs for mode '{mode}'"
        )

    return uris[:n_drones]


def connect_all(config, n_drones):
    """
    Open a link to each drone, one after another.

    Input:  config   — dictionary from load_config()
            n_drones — how many drones to connect to
    Output: list of SyncCrazyflie objects. drones[0] is CF1, drones[1] is
            CF2, and so on. Use drones[i].cf to send commands.

    If one connection fails, the links that already opened are closed
    before the error is passed on, so nothing is left half-connected.

    Called from: the start of every test and experiment.
    """
    # Load CFLib's radio/UDP drivers. Must happen once before any connection.
    cflib.crtp.init_drivers()

    drones = []

    try:
        for i, uri in enumerate(get_uris(config, n_drones)):
            print(f"Connecting to CF{i + 1} ({uri})...")

            # rw_cache stores the drone's parameter tables on disk so the
            # next connection is faster.
            cf = Crazyflie(rw_cache=config["cache_dir"])
            scf = SyncCrazyflie(uri, cf)
            scf.open_link()

            drones.append(scf)
            print(f"CF{i + 1} connected.")

    except Exception:
        disconnect_all(drones)
        raise

    return drones


def disconnect_all(drones):
    """
    Close the link to every drone.

    Input:  drones — list returned by connect_all()
    Output: none.

    Errors on one drone are ignored so the others still get closed.

    Called from: the `finally:` block at the end of every test and
    experiment, so it runs even after a crash or Ctrl-C.
    """
    for scf in drones:
        try:
            scf.close_link()
        except Exception:
            pass

    print("Connections closed.")
