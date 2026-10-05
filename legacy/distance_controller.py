import time
import csv
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


# ============================================================
# CONFIGURATION
# ============================================================

URIS = [
    "udp://127.0.0.1:19850",
    "udp://127.0.0.1:19851",
    "udp://127.0.0.1:19852",
]

NAMES = ["CF1", "CF2", "CF3"]

HEIGHT = 0.5

# Desired distance between neighbouring drones
DESIRED_DISTANCE = 0.5

# Controller gain
GAIN = 0.15

# Maximum movement per control step
MAX_STEP = 0.03

# Controller update period
CONTROL_PERIOD = 0.3

# Total experiment duration
EXPERIMENT_TIME = 15.0

# Data file
CSV_FILE = "distance_experiment.csv"


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def limit(value, minimum, maximum):
    return max(minimum, min(value, maximum))


# ============================================================
# MAIN
# ============================================================

def main():

    cflib.crtp.init_drivers()

    drones = []
    positions = {}

    # Store experiment data
    experiment_data = []


    # ========================================================
    # CONNECT TO DRONES
    # ========================================================

    for name, uri in zip(NAMES, URIS):

        print(f"Connecting to {name}...")

        cf = Crazyflie(rw_cache="./cache")

        scf = SyncCrazyflie(uri, cf)

        scf.open_link()

        drones.append(scf)

        print(f"{name} connected.")


    try:

        # ====================================================
        # TELEMETRY CALLBACK
        # ====================================================

        def create_callback(name):

            def callback(timestamp, data, logconf):

                positions[name] = {
                    "x": data["stateEstimate.x"],
                    "y": data["stateEstimate.y"],
                    "z": data["stateEstimate.z"],
                }

            return callback


        # ====================================================
        # START TELEMETRY
        # ====================================================

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
                create_callback(name)
            )

            scf.cf.log.add_config(log_config)

            log_config.start()

            log_configs.append(log_config)


        # ====================================================
        # WAIT FOR TELEMETRY
        # ====================================================

        print("\nWaiting for telemetry...")

        while not all(
            name in positions
            for name in NAMES
        ):

            time.sleep(0.1)

        print("Telemetry ready.")


        # ====================================================
        # TAKEOFF
        # ====================================================

        print("\nTaking off...")

        for scf in drones:

            scf.cf.high_level_commander.takeoff(
                HEIGHT,
                2.0
            )

        time.sleep(3)

        print("All drones hovering.")


        # ====================================================
        # INITIAL LINE FORMATION
        # ====================================================

        print("\nSetting initial formation...")

        initial_positions = {
            "CF1": 0.0,
            "CF2": 0.5,
            "CF3": 1.0,
        }

        for name, scf in zip(NAMES, drones):

            scf.cf.high_level_commander.go_to(
                initial_positions[name],
                0.0,
                HEIGHT,
                0.0,
                2.0
            )

        time.sleep(3)


        # ====================================================
        # DISTANCE CONTROLLER
        # ====================================================

        print("\nStarting distance controller...")

        print(
            f"Desired neighbour distance = "
            f"{DESIRED_DISTANCE:.2f} m"
        )

        print(
            "\n"
            "Time     d12      d23      "
            "S12      S23      U"
        )

        print("-" * 65)


        start_time = time.time()


        while True:

            elapsed = time.time() - start_time

            if elapsed >= EXPERIMENT_TIME:
                break


            # =================================================
            # READ CURRENT POSITIONS
            # =================================================

            cf1 = positions["CF1"]
            cf2 = positions["CF2"]
            cf3 = positions["CF3"]

            x1 = cf1["x"]
            x2 = cf2["x"]
            x3 = cf3["x"]


            # =================================================
            # CALCULATE NEIGHBOUR DISTANCES
            # =================================================

            d12 = abs(x2 - x1)

            d23 = abs(x3 - x2)


            # =================================================
            # CALCULATE SURPRISE / PREDICTION ERRORS
            # =================================================

            S12 = d12 - DESIRED_DISTANCE

            S23 = d23 - DESIRED_DISTANCE


            # =================================================
            # SURPRISE / POTENTIAL
            #
            # U = 1/2 * (S12^2 + S23^2)
            # =================================================

            U = 0.5 * (
                S12 ** 2
                + S23 ** 2
            )


            # =================================================
            # GRADIENT-DESCENT CONTROL
            # =================================================

            u1 = GAIN * S12

            u2 = (
                -GAIN * S12
                + GAIN * S23
            )

            u3 = -GAIN * S23


            # =================================================
            # LIMIT MOVEMENT
            # =================================================

            u1 = limit(
                u1,
                -MAX_STEP,
                MAX_STEP
            )

            u2 = limit(
                u2,
                -MAX_STEP,
                MAX_STEP
            )

            u3 = limit(
                u3,
                -MAX_STEP,
                MAX_STEP
            )


            # =================================================
            # NEW TARGET POSITIONS
            # =================================================

            target_x1 = x1 + u1

            target_x2 = x2 + u2

            target_x3 = x3 + u3


            # =================================================
            # SEND COMMANDS
            # =================================================

            drones[0].cf.high_level_commander.go_to(
                target_x1,
                0.0,
                HEIGHT,
                0.0,
                CONTROL_PERIOD
            )

            drones[1].cf.high_level_commander.go_to(
                target_x2,
                0.0,
                HEIGHT,
                0.0,
                CONTROL_PERIOD
            )

            drones[2].cf.high_level_commander.go_to(
                target_x3,
                0.0,
                HEIGHT,
                0.0,
                CONTROL_PERIOD
            )


            # =================================================
            # SAVE DATA
            # =================================================

            experiment_data.append([
                elapsed,
                d12,
                d23,
                S12,
                S23,
                U,
                x1,
                x2,
                x3,
            ])


            # =================================================
            # PRINT DATA
            # =================================================

            print(
                f"{elapsed:5.1f}s   "
                f"{d12:6.3f}   "
                f"{d23:6.3f}   "
                f"{S12:+6.3f}   "
                f"{S23:+6.3f}   "
                f"{U:7.5f}"
            )


            time.sleep(CONTROL_PERIOD)


        # ====================================================
        # CONTROLLER FINISHED
        # ====================================================

        print("\nController finished.")


        # ====================================================
        # SAVE CSV
        # ====================================================

        with open(
            CSV_FILE,
            "w",
            newline=""
        ) as file:

            writer = csv.writer(file)

            writer.writerow([
                "time",
                "d12",
                "d23",
                "S12",
                "S23",
                "U",
                "x1",
                "x2",
                "x3",
            ])

            writer.writerows(
                experiment_data
            )


        print(
            f"\nExperiment data saved to "
            f"{CSV_FILE}"
        )


        # ====================================================
        # FINAL MEASUREMENTS
        # ====================================================

        cf1 = positions["CF1"]
        cf2 = positions["CF2"]
        cf3 = positions["CF3"]

        x1 = cf1["x"]
        x2 = cf2["x"]
        x3 = cf3["x"]

        d12 = abs(x2 - x1)
        d23 = abs(x3 - x2)

        S12 = d12 - DESIRED_DISTANCE
        S23 = d23 - DESIRED_DISTANCE

        U = 0.5 * (
            S12 ** 2
            + S23 ** 2
        )


        # ====================================================
        # FINAL FORMATION
        # ====================================================

        print("\n==============================")
        print("FINAL FORMATION")
        print("==============================")

        print(f"CF1 x = {x1:.3f}")
        print(f"CF2 x = {x2:.3f}")
        print(f"CF3 x = {x3:.3f}")

        print()

        print(
            f"CF1 <-> CF2 = "
            f"{d12:.3f} m"
        )

        print(
            f"CF2 <-> CF3 = "
            f"{d23:.3f} m"
        )

        print()

        print(
            f"S12 = {S12:+.3f}"
        )

        print(
            f"S23 = {S23:+.3f}"
        )

        print(
            f"Final surprise U = "
            f"{U:.6f}"
        )


        # ====================================================
        # LAND
        # ====================================================

        print("\nLanding...")

        for scf in drones:

            scf.cf.high_level_commander.land(
                0.0,
                2.0
            )

        time.sleep(3)

        print("Landed.")


        # ====================================================
        # STOP TELEMETRY
        # ====================================================

        for log_config in log_configs:

            log_config.stop()


    finally:

        for drone in drones:

            drone.close_link()

        print("Connections closed.")


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":
    main()