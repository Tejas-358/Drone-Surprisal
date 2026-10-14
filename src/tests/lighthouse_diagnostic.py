"""
lighthouse_diagnostic.py — find out WHY a drone's Lighthouse position is wrong.
(NO flying: the motors are never started.)

For each drone it prints:
  1. whether the Lighthouse deck is detected and which estimator is running;
  2. the base-station GEOMETRY stored on the drone — where the drone
     believes each base station is. All drones must store the SAME
     geometry; if one differs, its positions will be shifted;
  3. for ~2 s: which base stations it receives, whether it has valid
     geometry/calibration for them, and its position.

When to use: a drone's position is wrong (e.g. z below 0 on the floor)
while other drones look fine.

Run from the repo root (cfclient CLOSED):
    python -m src.tests.lighthouse_diagnostic --mode real --drones 3
"""

import argparse
import threading
import time

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import LighthouseMemHelper

from src.utils.connection import load_config, connect_all, disconnect_all


def bits(value, n=4):
    """
    Turn a bitmask into a readable list of base-station numbers.

    Input:  value — integer where bit 0 = base station 1, bit 1 = BS 2, ...
            n     — how many base stations to check
    Output: e.g. "1 2" if bits 0 and 1 are set, or "none".

    Called from: read_status() to print the lighthouse.* log values.
    """
    stations = [str(i + 1) for i in range(n) if value & (1 << i)]
    return " ".join(stations) if stations else "none"


def read_geometry(scf):
    """
    Read the base-station geometry stored in the drone's memory.

    Input:  scf — one connected drone
    Output: dictionary {base station index: geometry object}, where each
            geometry has .origin (x, y, z of the base station, metres)
            and .valid (True/False). Empty if nothing could be read.

    Called from: main(), once per drone.
    """
    result = {}
    done = threading.Event()

    # CFLib reads memory in the background and calls this when finished.
    def callback(geometries):
        result.update(geometries)
        done.set()

    LighthouseMemHelper(scf.cf).read_all_geos(callback)
    done.wait(timeout=5.0)
    return result


def read_status(scf, seconds=2.0):
    """
    Stream Lighthouse status + position for a few seconds and print it.

    Input:  scf     — one connected drone
            seconds — how long to watch
    Output: none (prints one line every 0.5 s).

    Called from: main(), once per drone.
    """
    log = LogConfig(name="LhDiag", period_in_ms=500)
    log.add_variable("lighthouse.bsReceive", "uint16_t")  # BS whose light it sees now
    log.add_variable("lighthouse.bsGeoVal", "uint16_t")   # BS with valid geometry
    log.add_variable("lighthouse.bsCalVal", "uint16_t")   # BS with valid calibration
    log.add_variable("stateEstimate.x", "float")
    log.add_variable("stateEstimate.y", "float")
    log.add_variable("stateEstimate.z", "float")

    def callback(timestamp, data, logconf):
        print(f"   receiving BS: {bits(data['lighthouse.bsReceive']):9s} | "
              f"geometry OK: {bits(data['lighthouse.bsGeoVal']):9s} | "
              f"calibration OK: {bits(data['lighthouse.bsCalVal']):9s} | "
              f"x={data['stateEstimate.x']:6.2f} y={data['stateEstimate.y']:6.2f} "
              f"z={data['stateEstimate.z']:6.2f}")

    log.data_received_cb.add_callback(callback)
    scf.cf.log.add_config(log)
    log.start()
    time.sleep(seconds)
    log.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drones", type=int, default=3, help="how many drones to check")
    parser.add_argument("--mode", choices=["sim", "real"], help="override mode in config")
    args = parser.parse_args()

    config = load_config()
    if args.mode:
        config["mode"] = args.mode

    drones = connect_all(config, args.drones)

    try:
        for i, scf in enumerate(drones):
            get = scf.cf.param.get_value
            print(f"\n========== CF{i + 1} ==========")
            print(f"Lighthouse deck: {get('deck.bcLighthouse4')}   "
                  f"Estimator: {get('stabilizer.estimator')} (2 = Kalman, needed)")

            print("Stored base-station geometry (where this drone thinks each BS is):")
            geometries = read_geometry(scf)
            for bs, geo in sorted(geometries.items()):
                if geo.valid:
                    x, y, z = geo.origin
                    print(f"   BS{bs + 1}: x={x:6.2f}  y={y:6.2f}  z={z:6.2f}")
            if not any(g.valid for g in geometries.values()):
                print("   NONE — this drone has no geometry stored!")

            print("Live status (keep the drone still):")
            read_status(scf)

    finally:
        disconnect_all(drones)

    print("\nCompare the 'Stored base-station geometry' lines between drones.")
    print("They must be (almost) identical. A drone whose numbers differ needs")
    print("the configuration re-imported in cfclient (Lighthouse tab -> Import).")


if __name__ == "__main__":
    main()
