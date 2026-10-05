import time
import csv
import math

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
    "udp://127.0.0.1:19850",   # CF1
    "udp://127.0.0.1:19851",   # CF2
    "udp://127.0.0.1:19852",   # CF3
]

# Virtual wall
WALL_X = 2.0
WALL_DISTANCE = 0.5

# Desired distance between drones
FOLLOW_DISTANCE = 0.5

# Height
HEIGHT = 0.5

# CF1 speed
LEADER_SPEED = 0.15

# Maximum follower speed
MAX_FOLLOW_SPEED = 0.15

# Position-following gain
FOLLOW_GAIN = 0.8

# Experiment
EXPERIMENT_TIME = 25.0
CONTROL_PERIOD = 0.1

# ------------------------------------------------------------
# SURPRISE PARAMETERS
# ------------------------------------------------------------

# Noise scale N in:
#
# U = 0.5 * (S / N)^2
#
NOISE_SCALE = 0.05

# How strongly surprise suppresses movement
SURPRISE_GAIN = 2.0


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


# ============================================================
# TELEMETRY SETUP
# ============================================================

def setup_telemetry(scf):

    cf = scf.cf

    # Create LogConfig object
    log_config = LogConfig(
        name="StateEstimate",
        period_in_ms=100
    )

    # Add variables
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

    log_config.add_variable(
        "stateEstimate.vx",
        "float"
    )

    log_config.add_variable(
        "stateEstimate.vy",
        "float"
    )

    log_config.add_variable(
        "stateEstimate.vz",
        "float"
    )

    # Local storage for telemetry
    data = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
    }

    # Callback
    def log_callback(timestamp, received_data, logconf):

        data["x"] = received_data["stateEstimate.x"]
        data["y"] = received_data["stateEstimate.y"]
        data["z"] = received_data["stateEstimate.z"]

        data["vx"] = received_data["stateEstimate.vx"]
        data["vy"] = received_data["stateEstimate.vy"]
        data["vz"] = received_data["stateEstimate.vz"]

    log_config.data_received_cb.add_callback(
        log_callback
    )

    # Register config with Crazyflie
    cf.log.add_config(log_config)

    # Start logging
    log_config.start()

    return data, log_config


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 65)
    print("EXPERIMENT 2")
    print("SURPRISE-MINIMIZING CONTROLLER")
    print("=" * 65)

    # --------------------------------------------------------
    # INITIALIZE CFLIB
    # --------------------------------------------------------

    cflib.crtp.init_drivers()

    drones = []
    telemetry = []

    # --------------------------------------------------------
    # CONNECT TO THREE DRONES
    # --------------------------------------------------------

    for i, uri in enumerate(URIS):

        print(f"Connecting to CF{i + 1}...")

        cf = Crazyflie(
            rw_cache="./cache"
        )

        scf = SyncCrazyflie(
            uri,
            cf
        )

        scf.open_link()

        drones.append(scf)

        print(f"CF{i + 1} connected.")

    # --------------------------------------------------------
    # TELEMETRY
    # --------------------------------------------------------

    print("\nSetting up telemetry...")

    for scf in drones:

        data, log_config = setup_telemetry(scf)

        telemetry.append({
            "data": data,
            "log": log_config
        })

    # Give telemetry time to start
    time.sleep(2)

    print("Telemetry ready.")

    # --------------------------------------------------------
    # TAKEOFF
    # --------------------------------------------------------

    print("\nTaking off...")

    for scf in drones:

        scf.cf.high_level_commander.takeoff(
            HEIGHT,
            2.0
        )

    time.sleep(3)

    # --------------------------------------------------------
    # INITIAL FORMATION
    #
    # CF1 = 0.0
    # CF2 = -0.5
    # CF3 = -1.0
    #
    # All drones are in a line.
    # --------------------------------------------------------

    print("Creating initial formation...")

    formation = [
        (0.0, 0.0, HEIGHT),
        (-0.5, 0.0, HEIGHT),
        (-1.0, 0.0, HEIGHT),
    ]

    for scf, position in zip(drones, formation):

        scf.cf.high_level_commander.go_to(
            position[0],
            position[1],
            position[2],
            0.0,
            2.0
        )

    time.sleep(3)

    print("Formation ready.")

    # --------------------------------------------------------
    # CSV
    # --------------------------------------------------------

    csv_file = open(
        "surprise_minimization_experiment.csv",
        "w",
        newline=""
    )

    writer = csv.writer(csv_file)

    writer.writerow([
        "time",

        "x1",
        "x2",
        "x3",

        "v1",
        "v2",
        "v3",

        "wall_distance",

        "distance_12",
        "distance_23",

        "predicted_v2",
        "predicted_v3",

        "S2",
        "S3",

        "U2",
        "U3",

        "surprise_factor_2",
        "surprise_factor_3",

        "v2_command",
        "v3_command",

        "cf1_stopped"
    ])

    # --------------------------------------------------------
    # START EXPERIMENT
    # --------------------------------------------------------

    print("\n" + "=" * 65)
    print("STARTING EXPERIMENT")
    print("=" * 65)

    start_time = time.time()

    cf1_stopped = False

    last_print = -1

    try:

        while True:

            elapsed = time.time() - start_time

            # ------------------------------------------------
            # END CONDITION
            # ------------------------------------------------

            if elapsed >= EXPERIMENT_TIME:

                break

            # ------------------------------------------------
            # READ TELEMETRY
            # ------------------------------------------------

            x1 = telemetry[0]["data"]["x"]
            x2 = telemetry[1]["data"]["x"]
            x3 = telemetry[2]["data"]["x"]

            v1 = telemetry[0]["data"]["vx"]
            v2 = telemetry[1]["data"]["vx"]
            v3 = telemetry[2]["data"]["vx"]

            # ------------------------------------------------
            # WALL DISTANCE
            # ------------------------------------------------

            wall_distance = WALL_X - x1

            # =================================================
            # CF1
            # =================================================

            if not cf1_stopped:

                if wall_distance <= WALL_DISTANCE:

                    print(
                        "\n>>> CF1 REACHED WALL -> STOPPING <<<"
                    )

                    cf1_stopped = True

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

            else:

                drones[0].cf.commander.send_velocity_world_setpoint(
                    0.0,
                    0.0,
                    0.0,
                    0.0
                )

            # =================================================
            # CF2
            # =================================================
            #
            # Prediction:
            #
            #   predicted_v2 = v1
            #
            # CF2 expects its velocity to resemble CF1.
            #
            # Prediction error:
            #
            #   S2 = predicted_v2 - v2
            #
            # Surprise:
            #
            #   U2 = 0.5 * (S2 / N)^2
            #
            # Surprise then suppresses CF2's movement.
            #
            # =================================================

            predicted_v2 = v1

            S2 = predicted_v2 - v2

            U2 = 0.5 * (
                S2 / NOISE_SCALE
            ) ** 2

            # ------------------------------------------------
            # NORMAL FORMATION CONTROL
            # ------------------------------------------------

            desired_x2 = x1 - FOLLOW_DISTANCE

            position_error_2 = desired_x2 - x2

            base_v2 = (
                FOLLOW_GAIN *
                position_error_2
            )

            base_v2 = clamp(
                base_v2,
                -MAX_FOLLOW_SPEED,
                MAX_FOLLOW_SPEED
            )

            # ------------------------------------------------
            # SURPRISE MINIMIZATION
            #
            # Large U -> small movement
            #
            # Small U -> normal movement
            # ------------------------------------------------

            surprise_factor_2 = math.exp(
                -SURPRISE_GAIN * U2
            )

            v2_command = (
                base_v2 *
                surprise_factor_2
            )

            # ------------------------------------------------
            # SEND CF2 COMMAND
            # ------------------------------------------------

            drones[1].cf.commander.send_velocity_world_setpoint(
                v2_command,
                0.0,
                0.0,
                0.0
            )

            # =================================================
            # CF3
            # =================================================

            predicted_v3 = v2

            S3 = predicted_v3 - v3

            U3 = 0.5 * (
                S3 / NOISE_SCALE
            ) ** 2

            # ------------------------------------------------
            # NORMAL FORMATION CONTROL
            # ------------------------------------------------

            desired_x3 = x2 - FOLLOW_DISTANCE

            position_error_3 = desired_x3 - x3

            base_v3 = (
                FOLLOW_GAIN *
                position_error_3
            )

            base_v3 = clamp(
                base_v3,
                -MAX_FOLLOW_SPEED,
                MAX_FOLLOW_SPEED
            )

            # ------------------------------------------------
            # SURPRISE MINIMIZATION
            # ------------------------------------------------

            surprise_factor_3 = math.exp(
                -SURPRISE_GAIN * U3
            )

            v3_command = (
                base_v3 *
                surprise_factor_3
            )

            # ------------------------------------------------
            # SEND CF3 COMMAND
            # ------------------------------------------------

            drones[2].cf.commander.send_velocity_world_setpoint(
                v3_command,
                0.0,
                0.0,
                0.0
            )

            # =================================================
            # DISTANCES
            # =================================================

            distance_12 = abs(x1 - x2)

            distance_23 = abs(x2 - x3)

            # =================================================
            # PRINT ONCE PER SECOND
            # =================================================

            current_second = int(elapsed)

            if current_second != last_print:

                last_print = current_second

                print(
                    f"t={elapsed:5.1f}s | "
                    f"x=[{x1:5.2f}, "
                    f"{x2:5.2f}, "
                    f"{x3:5.2f}] | "
                    f"v=[{v1:5.2f}, "
                    f"{v2:5.2f}, "
                    f"{v3:5.2f}] | "
                    f"U=[{U2:6.3f}, "
                    f"{U3:6.3f}]"
                )

            # =================================================
            # SAVE DATA
            # =================================================

            writer.writerow([
                elapsed,

                x1,
                x2,
                x3,

                v1,
                v2,
                v3,

                wall_distance,

                distance_12,
                distance_23,

                predicted_v2,
                predicted_v3,

                S2,
                S3,

                U2,
                U3,

                surprise_factor_2,
                surprise_factor_3,

                v2_command,
                v3_command,

                cf1_stopped
            ])

            csv_file.flush()

            # ------------------------------------------------
            # CONTROL RATE
            # ------------------------------------------------

            time.sleep(CONTROL_PERIOD)

    finally:

        # ====================================================
        # STOP HORIZONTAL MOTION
        # ====================================================

        print("\nStopping horizontal motion...")

        for scf in drones:

            scf.cf.commander.send_velocity_world_setpoint(
                0.0,
                0.0,
                0.0,
                0.0
            )

        time.sleep(1)

        # ====================================================
        # LAND
        # ====================================================

        print("Landing...")

        for scf in drones:

            scf.cf.high_level_commander.land(
                0.0,
                2.0
            )

        time.sleep(3)

        # ====================================================
        # STOP TELEMETRY
        # ====================================================

        for item in telemetry:

            try:
                item["log"].stop()
            except Exception:
                pass

        # ====================================================
        # CLOSE CONNECTIONS
        # ====================================================

        for scf in drones:

            try:
                scf.close_link()
            except Exception:
                pass

        csv_file.close()

        print("\nExperiment finished.")
        print(
            "Data saved to "
            "surprise_minimization_experiment.csv"
        )


if __name__ == "__main__":
    main()