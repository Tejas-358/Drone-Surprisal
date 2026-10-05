import time
import csv
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

import warnings
warnings.filterwarnings("ignore")


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
FOLLOW_DISTANCE = 0.5

# Virtual wall position
WALL_X = 2.0

# CF1 stops this distance from wall
WALL_DISTANCE = 0.5

# CF1 forward speed
LEADER_SPEED = 0.15

# Maximum follower speed
FOLLOW_SPEED_MAX = 0.15

# Controller gain for followers
FOLLOW_GAIN = 0.8

# Experiment duration
EXPERIMENT_TIME = 25.0

# Control period
CONTROL_PERIOD = 0.1

# Stop threshold
STOP_VELOCITY = 0.02

# CSV file
CSV_FILE = "wall_follow_experiment.csv"


# ============================================================
# HELPER
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

    previous_positions = {}

    experiment_data = []


    # ========================================================
    # CONNECT
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
        # INITIAL FORMATION
        # ====================================================

        print("\nSetting initial formation...")

        initial_positions = {
            "CF1": 0.0,
            "CF2": -0.5,
            "CF3": -1.0,
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
        # INITIAL POSITION HISTORY
        # ====================================================

        for name in NAMES:

            previous_positions[name] = positions[name]["x"]


        # ====================================================
        # EXPERIMENT
        # ====================================================

        print("\n========================================")
        print("STARTING WALL-FOLLOWING EXPERIMENT")
        print("========================================")

        print(
            f"Wall position       : {WALL_X:.2f} m"
        )

        print(
            f"Wall stop distance  : {WALL_DISTANCE:.2f} m"
        )

        print(
            f"Follower distance   : {FOLLOW_DISTANCE:.2f} m"
        )

        print("\n")

        print(
            "Time   "
            "x1     "
            "x2     "
            "x3     "
            "v1     "
            "v2     "
            "v3     "
            "E12    "
            "E23    "
            "S2     "
            "S3     "
            "U2     "
            "U3"
        )

        print("-" * 110)


        start_time = time.time()

        cf1_stopped = False

        cf2_stopped = False

        cf3_stopped = False


        while True:

            elapsed = time.time() - start_time

            if elapsed >= EXPERIMENT_TIME:

                break


            # =================================================
            # POSITIONS
            # =================================================

            x1 = positions["CF1"]["x"]

            x2 = positions["CF2"]["x"]

            x3 = positions["CF3"]["x"]


            # =================================================
            # VELOCITIES
            # =================================================

            v1 = (
                x1 - previous_positions["CF1"]
            ) / CONTROL_PERIOD

            v2 = (
                x2 - previous_positions["CF2"]
            ) / CONTROL_PERIOD

            v3 = (
                x3 - previous_positions["CF3"]
            ) / CONTROL_PERIOD


            # =================================================
            # WALL DISTANCE
            # =================================================

            wall_distance = WALL_X - x1


            # =================================================
            # CF1: MOVE TOWARD WALL
            # =================================================

            if not cf1_stopped:

                if wall_distance <= WALL_DISTANCE:

                    cf1_stopped = True

                    print(
                        "\n>>> CF1 REACHED WALL STOP DISTANCE <<<"
                    )

                    drones[0].cf.commander.send_velocity_world_setpoint(
                        0.0,
                        0.0,
                        0.0,
                        0.0
                    )

                else:

                    drones[0].cf.commander.send_velocity_world_setpoint(
                        LEADER_SPEED,
                        0.0,
                        0.0,
                        0.0
                    )


            # =================================================
            # CF2 FOLLOWING ERROR
            # =================================================

            desired_x2 = x1 - FOLLOW_DISTANCE

            E12 = desired_x2 - x2


            # =================================================
            # CF3 FOLLOWING ERROR
            # =================================================

            desired_x3 = x2 - FOLLOW_DISTANCE

            E23 = desired_x3 - x3


            # =================================================
            # CF2 SURPRISE
            #
            # CF2 predicts CF1's motion.
            # =================================================

            S2 = v1 - v2

            U2 = 0.5 * S2 ** 2


            # =================================================
            # CF3 SURPRISE
            # =================================================

            S3 = v2 - v3

            U3 = 0.5 * S3 ** 2


            # =================================================
            # CF2 FOLLOWING CONTROL
            # =================================================

            if not cf2_stopped:

                if cf1_stopped and abs(v1) < STOP_VELOCITY:

                    if abs(E12) < 0.08 and abs(v2) < STOP_VELOCITY:

                        cf2_stopped = True

                        print(
                            "\n>>> CF2 STOPPED AFTER CF1 <<<"
                        )

                        drones[1].cf.commander.send_velocity_world_setpoint(
                            0.0,
                            0.0,
                            0.0,
                            0.0
                        )

                    else:

                        v2_command = FOLLOW_GAIN * E12

                        v2_command = limit(
                            v2_command,
                            -FOLLOW_SPEED_MAX,
                            FOLLOW_SPEED_MAX
                        )

                        drones[1].cf.commander.send_velocity_world_setpoint(
                            v2_command,
                            0.0,
                            0.0,
                            0.0
                        )

                else:

                    v2_command = FOLLOW_GAIN * E12

                    v2_command = limit(
                        v2_command,
                        -FOLLOW_SPEED_MAX,
                        FOLLOW_SPEED_MAX
                    )

                    drones[1].cf.commander.send_velocity_world_setpoint(
                        v2_command,
                        0.0,
                        0.0,
                        0.0
                    )


            # =================================================
            # CF3 FOLLOWING CONTROL
            # =================================================

            if not cf3_stopped:

                if cf2_stopped and abs(v2) < STOP_VELOCITY:

                    if abs(E23) < 0.08 and abs(v3) < STOP_VELOCITY:

                        cf3_stopped = True

                        print(
                            "\n>>> CF3 STOPPED AFTER CF2 <<<"
                        )

                        drones[2].cf.commander.send_velocity_world_setpoint(
                            0.0,
                            0.0,
                            0.0,
                            0.0
                        )

                    else:

                        v3_command = FOLLOW_GAIN * E23

                        v3_command = limit(
                            v3_command,
                            -FOLLOW_SPEED_MAX,
                            FOLLOW_SPEED_MAX
                        )

                        drones[2].cf.commander.send_velocity_world_setpoint(
                            v3_command,
                            0.0,
                            0.0,
                            0.0
                        )

                else:

                    v3_command = FOLLOW_GAIN * E23

                    v3_command = limit(
                        v3_command,
                        -FOLLOW_SPEED_MAX,
                        FOLLOW_SPEED_MAX
                    )

                    drones[2].cf.commander.send_velocity_world_setpoint(
                        v3_command,
                        0.0,
                        0.0,
                        0.0
                    )


            # =================================================
            # SAVE DATA
            # =================================================

            experiment_data.append([
                elapsed,
                x1,
                x2,
                x3,
                v1,
                v2,
                v3,
                wall_distance,
                E12,
                E23,
                S2,
                S3,
                U2,
                U3,
                int(cf1_stopped),
                int(cf2_stopped),
                int(cf3_stopped),
            ])


            # =================================================
            # PRINT
            # =================================================

            print(
                f"{elapsed:5.1f} "
                f"{x1:6.2f} "
                f"{x2:6.2f} "
                f"{x3:6.2f} "
                f"{v1:+6.2f} "
                f"{v2:+6.2f} "
                f"{v3:+6.2f} "
                f"{E12:+6.2f} "
                f"{E23:+6.2f} "
                f"{S2:+6.2f} "
                f"{S3:+6.2f} "
                f"{U2:6.3f} "
                f"{U3:6.3f}"
            )


            # =================================================
            # POSITION HISTORY
            # =================================================

            previous_positions["CF1"] = x1

            previous_positions["CF2"] = x2

            previous_positions["CF3"] = x3


            time.sleep(CONTROL_PERIOD)


        # ====================================================
        # STOP ALL VELOCITIES
        # ====================================================

        for scf in drones:

            scf.cf.commander.send_velocity_world_setpoint(
                0.0,
                0.0,
                0.0,
                0.0
            )


        # ====================================================
        # SAVE DATA
        # ====================================================

        with open(
            CSV_FILE,
            "w",
            newline=""
        ) as file:

            writer = csv.writer(file)

            writer.writerow([
                "time",
                "x1",
                "x2",
                "x3",
                "v1",
                "v2",
                "v3",
                "wall_distance",
                "E12",
                "E23",
                "S2",
                "S3",
                "U2",
                "U3",
                "CF1_stopped",
                "CF2_stopped",
                "CF3_stopped",
            ])

            writer.writerows(
                experiment_data
            )


        # ====================================================
        # FINAL RESULT
        # ====================================================

        print("\n========================================")
        print("EXPERIMENT FINISHED")
        print("========================================")

        print(
            f"Final CF1 position : {x1:.3f} m"
        )

        print(
            f"Final CF2 position : {x2:.3f} m"
        )

        print(
            f"Final CF3 position : {x3:.3f} m"
        )

        print(
            f"\nFinal wall distance: "
            f"{wall_distance:.3f} m"
        )

        print(
            f"\nCF1 stopped: {cf1_stopped}"
        )

        print(
            f"CF2 stopped: {cf2_stopped}"
        )

        print(
            f"CF3 stopped: {cf3_stopped}"
        )

        print(
            f"\nFinal CF2 surprise U2: "
            f"{U2:.6f}"
        )

        print(
            f"Final CF3 surprise U3: "
            f"{U3:.6f}"
        )

        print(
            f"\nData saved to {CSV_FILE}"
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
        # STOP LOGGING
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