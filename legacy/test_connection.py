import time
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
import math


def distance_between(p1, p2):
    dx = p1["x"] - p2["x"]
    dy = p1["y"] - p2["y"]
    dz = p1["z"] - p2["z"]

    return math.sqrt(
        dx**2 + dy**2 + dz**2
    )


URIS = [
    "udp://127.0.0.1:19850",
    "udp://127.0.0.1:19851",
    "udp://127.0.0.1:19852",
]

NAMES = [
    "CF1",
    "CF2",
    "CF3",
]


def main():

    cflib.crtp.init_drivers()

    drones = []
    latest_positions = {}

    try:

        # --------------------------------------------------
        # Connect to all three Crazyflies
        # --------------------------------------------------

        for name, uri in zip(NAMES, URIS):

            print(f"Connecting to {name}: {uri}")

            cf = Crazyflie(rw_cache="./cache")

            scf = SyncCrazyflie(uri, cf)

            scf.open_link()

            drones.append(scf)

            print(f"Connected to {name}")

        print()
        print("==============================")
        print("ALL THREE CRAZYFLIES CONNECTED")
        print("==============================")
        print()

        cf1 = drones[0].cf

        # --------------------------------------------------
        # Callback for receiving telemetry
        # --------------------------------------------------

        def create_position_callback(name):

            def position_callback(timestamp, data, logconf):

                latest_positions[name] = {
                    "x": data["stateEstimate.x"],
                    "y": data["stateEstimate.y"],
                    "z": data["stateEstimate.z"],
                }

            return position_callback

        # --------------------------------------------------
        # Create log configurations
        # --------------------------------------------------

        print("Taking off CF1...")

        cf1 = drones[0].cf

        # Increase thrust gradually
        for thrust in range(20000, 40001, 2000):
            cf1.commander.send_setpoint(
                0,      # roll
                0,      # pitch
                0,      # yaw rate
                thrust  # thrust
            )
            time.sleep(0.1)

        print("CF1 should now be hovering.")

        log_configs = []

        for name, scf in zip(NAMES, drones):

            log_config = LogConfig(
                name=f"Position_{name}",
                period_in_ms=100
            )

            log_config.add_variable("stateEstimate.x", "float")
            log_config.add_variable("stateEstimate.y", "float")
            log_config.add_variable("stateEstimate.z", "float")

            log_config.data_received_cb.add_callback(
                create_position_callback(name)
            )

            scf.cf.log.add_config(log_config)

            log_configs.append(log_config)

        # --------------------------------------------------
        # Start telemetry
        # --------------------------------------------------

        for log_config in log_configs:
            log_config.start()

        print("Reading positions...\n")

        # --------------------------------------------------
        # Print positions for 5 seconds
        # --------------------------------------------------

        start_time = time.time()

        while time.time() - start_time < 5:

            print("\033[2J\033[H", end="")

            print("==============================")
            print("CRAZYFLIE POSITIONS")
            print("==============================")

            for name in NAMES:

                if name in latest_positions:

                    p = latest_positions[name]

                    print(
                        f"{name}: "
                        f"x={p['x']:.3f}  "
                        f"y={p['y']:.3f}  "
                        f"z={p['z']:.3f}"
                    )

                else:

                    print(f"{name}: waiting for telemetry...")

            time.sleep(0.2)

            if all(name in latest_positions for name in NAMES):

                d12 = distance_between(
                    latest_positions["CF1"],
                    latest_positions["CF2"]
                )

                d23 = distance_between(
                    latest_positions["CF2"],
                    latest_positions["CF3"]
                )

                print()
                print("==============================")
                print("NEIGHBOUR DISTANCES")
                print("==============================")
                print(f"CF1 <-> CF2: {d12:.3f} m")
                print(f"CF2 <-> CF3: {d23:.3f} m")

        # --------------------------------------------------
        # Stop logging
        # --------------------------------------------------

        for log_config in log_configs:
            log_config.stop()

    finally:

        for drone in drones:
            drone.close_link()

        print("\nConnections closed.")


if __name__ == "__main__":
    main()