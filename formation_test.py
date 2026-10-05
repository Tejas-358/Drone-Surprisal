import time
import math
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


# ============================================================
# CONFIGURATION
# ============================================================

URIS = [
    "udp://127.0.0.1:19850",  # CF1
    "udp://127.0.0.1:19851",  # CF2
    "udp://127.0.0.1:19852",  # CF3
]

NAMES = ["CF1", "CF2", "CF3"]

# Desired height
HEIGHT = 0.5

# Desired distance between neighbouring drones
DISTANCE = 0.5

# Desired formation:
#
# CF1 ----0.5m---- CF2 ----0.5m---- CF3
#
TARGETS = {
    "CF1": (0.0, 0.0, HEIGHT),
    "CF2": (DISTANCE, 0.0, HEIGHT),
    "CF3": (2 * DISTANCE, 0.0, HEIGHT),
}


# ============================================================
# DISTANCE FUNCTION
# ============================================================

def distance(p1, p2):

    dx = p1["x"] - p2["x"]
    dy = p1["y"] - p2["y"]
    dz = p1["z"] - p2["z"]

    return math.sqrt(
        dx * dx +
        dy * dy +
        dz * dz
    )


# ============================================================
# MAIN
# ============================================================

def main():

    cflib.crtp.init_drivers()

    drones = []

    latest_positions = {}

    # --------------------------------------------------------
    # CONNECT TO ALL THREE DRONES
    # --------------------------------------------------------

    for name, uri in zip(NAMES, URIS):

        print(f"Connecting to {name}...")

        cf = Crazyflie(rw_cache="./cache")

        scf = SyncCrazyflie(uri, cf)

        scf.open_link()

        drones.append(scf)

        print(f"{name} connected.")

    try:

        # ----------------------------------------------------
        # POSITION CALLBACK
        # ----------------------------------------------------

        def create_position_callback(name):

            def position_callback(timestamp, data, logconf):

                latest_positions[name] = {
                    "x": data["stateEstimate.x"],
                    "y": data["stateEstimate.y"],
                    "z": data["stateEstimate.z"],
                }

            return position_callback


        # ----------------------------------------------------
        # START TELEMETRY
        # ----------------------------------------------------

        log_configs = []

        for name, scf in zip(NAMES, drones):

            log_config = LogConfig(
                name=f"Position_{name}",
                period_in_ms=100
            )

            log_config.add_variable(
                "stateEstimate.x",
                "float"
            )

            log_config.add_variable(
                "stateEstimate.y",
                "float"
            )

            log_config.add_variable(
                "stateEstimate.z",
                "float"
            )

            log_config.data_received_cb.add_callback(
                create_position_callback(name)
            )

            scf.cf.log.add_config(log_config)

            log_config.start()

            log_configs.append(log_config)


        # Give telemetry some time to arrive
        print("\nWaiting for position telemetry...")

        while not all(
            name in latest_positions
            for name in NAMES
        ):

            time.sleep(0.1)


        print("Telemetry received.")


        # ----------------------------------------------------
        # TAKEOFF
        # ----------------------------------------------------

        print("\nTaking off all three drones...")

        for scf in drones:

            scf.cf.high_level_commander.takeoff(
                HEIGHT,
                2.0
            )

        time.sleep(3)

        print("All drones should now be hovering.")


        # ----------------------------------------------------
        # MOVE INTO LINE FORMATION
        # ----------------------------------------------------

        print("\nMoving into line formation...")

        for name, scf in zip(NAMES, drones):

            x, y, z = TARGETS[name]

            print(
                f"{name} -> "
                f"x={x:.2f}, "
                f"y={y:.2f}, "
                f"z={z:.2f}"
            )

            scf.cf.high_level_commander.go_to(
                x,
                y,
                z,
                0.0,
                3.0
            )


        # ----------------------------------------------------
        # WAIT FOR FORMATION
        # ----------------------------------------------------

        time.sleep(5)


        # ----------------------------------------------------
        # PRINT FINAL POSITIONS
        # ----------------------------------------------------

        print("\n==============================")
        print("FINAL POSITIONS")
        print("==============================")

        for name in NAMES:

            p = latest_positions[name]

            print(
                f"{name}: "
                f"x={p['x']:.3f} "
                f"y={p['y']:.3f} "
                f"z={p['z']:.3f}"
            )


        # ----------------------------------------------------
        # CALCULATE NEIGHBOUR DISTANCES
        # ----------------------------------------------------

        d12 = distance(
            latest_positions["CF1"],
            latest_positions["CF2"]
        )

        d23 = distance(
            latest_positions["CF2"],
            latest_positions["CF3"]
        )

        print("\n==============================")
        print("NEIGHBOUR DISTANCES")
        print("==============================")

        print(
            f"CF1 <-> CF2: {d12:.3f} m"
        )

        print(
            f"CF2 <-> CF3: {d23:.3f} m"
        )

        print(
            f"\nDesired distance: {DISTANCE:.3f} m"
        )


        # ----------------------------------------------------
        # HOLD FORMATION
        # ----------------------------------------------------

        print("\nFormation complete.")

        print("Holding for 10 seconds...")

        time.sleep(10)


        # ----------------------------------------------------
        # LAND
        # ----------------------------------------------------

        print("\nLanding...")

        for scf in drones:

            scf.cf.high_level_commander.land(
                0.0,
                2.0
            )

        time.sleep(3)

        print("All drones landed.")


        # ----------------------------------------------------
        # STOP LOGGING
        # ----------------------------------------------------

        for log_config in log_configs:

            log_config.stop()


    finally:

        # ----------------------------------------------------
        # CLOSE CONNECTIONS
        # ----------------------------------------------------

        for drone in drones:

            drone.close_link()

        print("\nConnections closed.")


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    main()