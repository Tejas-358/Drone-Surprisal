
import os
import time
import threading
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

import warnings
warnings.filterwarnings("ignore")


# ============================================================
# 1. PARAMETERS
# ============================================================

N = 3

URIS = [
    "udp://127.0.0.1:19850",
    "udp://127.0.0.1:19851",
    "udp://127.0.0.1:19852"
]

# Virtual mass-spring-damper parameters
m = 1.0
k = 2.0
c = 1.5

# Formation target parameters
kt = 0.8
ct = 1.5

# Desired distance between drones
d_desired = 0.6

# Safety parameters
min_separation = 0.35
k_repulsion = 3.0

# Flight parameters
TAKEOFF_HEIGHT = 0.5
MAX_SPEED = 0.3
MAX_VERTICAL_SPEED = 0.2

# Target position in Gazebo world coordinates
target = np.array([1.0, 1.0])

# Simulation parameters
DT = 0.05
DURATION = 30.0

# Altitude controller
KZ = 1.5

# ============================================================
# 2. DRONE STATE STORAGE
# ============================================================

# Position: x, y, z
positions = np.zeros((N, 3))

# Velocity: vx, vy, vz
velocities = np.zeros((N, 3))

# Protect telemetry shared between callback threads and controller
state_lock = threading.Lock()

# Track which drones have received telemetry
received_state = [False] * N


# ============================================================
# 3. TELEMETRY CALLBACK
# ============================================================

def state_callback(index, timestamp, data, logconf):

    with state_lock:

        positions[index] = [
            data["stateEstimate.x"],
            data["stateEstimate.y"],
            data["stateEstimate.z"]
        ]

        velocities[index] = [
            data["stateEstimate.vx"],
            data["stateEstimate.vy"],
            data["stateEstimate.vz"]
        ]

        received_state[index] = True


# ============================================================
# 4. CONNECT TO CRAZYFLIE DRONES
# ============================================================

def connect_drones():

    cflib.crtp.init_drivers()

    connections = []
    log_configs = []

    for i in range(N):

        print(f"Connecting Drone {i+1}...")

        cf = Crazyflie(rw_cache="./cache")

        scf = SyncCrazyflie(URIS[i], cf=cf)

        scf.open_link()

        connections.append(scf)

        # Configure telemetry
        logconf = LogConfig(
            name=f"Drone{i+1}State",
            period_in_ms=50
        )

        logconf.add_variable("stateEstimate.x", "float")
        logconf.add_variable("stateEstimate.y", "float")
        logconf.add_variable("stateEstimate.z", "float")

        logconf.add_variable("stateEstimate.vx", "float")
        logconf.add_variable("stateEstimate.vy", "float")
        logconf.add_variable("stateEstimate.vz", "float")

        cf.log.add_config(logconf)

        logconf.data_received_cb.add_callback(
            lambda timestamp, data, logconf, index=i:
            state_callback(index, timestamp, data, logconf)
        )

        logconf.start()

        log_configs.append(logconf)

        print(f"Drone {i+1} connected.")

    return connections, log_configs


# ============================================================
# 5. WAIT FOR TELEMETRY
# ============================================================

def wait_for_telemetry():

    print("\nWaiting for drone telemetry...")

    start = time.time()

    while time.time() - start < 10:

        with state_lock:
            all_ready = all(received_state)

        if all_ready:
            print("Telemetry received from all drones.")
            return

        time.sleep(0.1)

    raise TimeoutError("Telemetry not received from all drones.")


# ============================================================
# 6. TAKEOFF
# ============================================================

def takeoff_drones(connections):

    print("\nTaking off...")

    for scf in connections:

        scf.cf.high_level_commander.takeoff(
            TAKEOFF_HEIGHT,
            2.0
        )

    time.sleep(3)

    print("Takeoff complete.")


# ============================================================
# 7. LAGRANGIAN FORMATION CONTROLLER
# ============================================================

def calculate_forces(q, v):

    # q: positions of three drones (x, y)
    # v: velocities of three drones (vx, vy)

    F = np.zeros((N, 2))

    # --------------------------------------------------------
    # A. FORMATION CENTRE
    # --------------------------------------------------------

    center = np.mean(q, axis=0)

    center_velocity = np.mean(v, axis=0)

    # --------------------------------------------------------
    # B. PAIRWISE SPRING AND DAMPING FORCES
    # --------------------------------------------------------

    for i in range(N):

        for j in range(i + 1, N):

            relative_position = q[i] - q[j]

            distance = np.linalg.norm(relative_position)

            if distance < 1e-6:
                continue

            direction = relative_position / distance

            # Distance error
            error = distance - d_desired

            # Virtual spring force
            F_spring = -k * error * direction

            # Relative velocity
            relative_velocity = v[i] - v[j]

            # Virtual damping force
            F_damping = -c * relative_velocity

            # Combined interaction force
            F_pair = F_spring + F_damping

            F[i] += F_pair
            F[j] -= F_pair

            # ------------------------------------------------
            # C. COLLISION AVOIDANCE
            # ------------------------------------------------

            if distance < min_separation:

                F_repulsion = (
                    k_repulsion
                    * (min_separation - distance)
                    * direction
                )

                F[i] += F_repulsion
                F[j] -= F_repulsion

    # --------------------------------------------------------
    # D. TARGET ATTRACTION
    # --------------------------------------------------------

    for i in range(N):

        F_target = -(kt / N) * (center - target)

        F_target_damping = -(ct / N) * center_velocity

        F[i] += F_target + F_target_damping

    return F


# ============================================================
# 8. VELOCITY LIMIT
# ============================================================

def limit_velocity(velocity):

    speed = np.linalg.norm(velocity)

    if speed > MAX_SPEED:

        velocity = velocity * (MAX_SPEED / speed)

    return velocity


# ============================================================
# 9. MAIN FORMATION CONTROL LOOP
# ============================================================

def run_formation(connections):

    print("\nStarting Lagrangian formation control...")

    # Virtual velocity commands
    velocity_commands = np.zeros((N, 2))

    # Initialize from measured velocities
    with state_lock:
        velocity_commands[:] = velocities[:, :2]

    start_time = time.time()

    iteration = 0

    while time.time() - start_time < DURATION:

        loop_start = time.time()

        # ----------------------------------------------------
        # A. READ ACTUAL DRONE STATES
        # ----------------------------------------------------

        with state_lock:

            q = positions[:, :2].copy()

            v = velocities[:, :2].copy()

            z = positions[:, 2].copy()

        # ----------------------------------------------------
        # B. CALCULATE LAGRANGIAN FORCES
        # ----------------------------------------------------

        F = calculate_forces(q, v)

        # ----------------------------------------------------
        # C. EQUATIONS OF MOTION
        # ----------------------------------------------------

        acceleration = F / m

        # ----------------------------------------------------
        # D. INTEGRATE ACCELERATION INTO VELOCITY COMMANDS
        # ----------------------------------------------------

        velocity_commands += acceleration * DT

        # ----------------------------------------------------
        # E. SEND COMMANDS TO EACH DRONE
        # ----------------------------------------------------

        for i in range(N):

            velocity_commands[i] = limit_velocity(
                velocity_commands[i]
            )

            # Horizontal velocity commands
            vx = velocity_commands[i, 0]

            vy = velocity_commands[i, 1]

            # Maintain takeoff altitude
            vz = KZ * (TAKEOFF_HEIGHT - z[i])

            vz = np.clip(
                vz,
                -MAX_VERTICAL_SPEED,
                MAX_VERTICAL_SPEED
            )

            connections[i].cf.commander.send_velocity_world_setpoint(
                float(vx),
                float(vy),
                float(vz),
                0.0
            )

        # ----------------------------------------------------
        # F. DISPLAY SYSTEM STATE
        # ----------------------------------------------------

        center = np.mean(q, axis=0)

        if iteration % 10 == 0:

            d12 = np.linalg.norm(q[0] - q[1])

            d23 = np.linalg.norm(q[1] - q[2])

            d31 = np.linalg.norm(q[2] - q[0])

            print(
                f"Center: {center.round(2)} | "
                f"D12: {d12:.2f} | "
                f"D23: {d23:.2f} | "
                f"D31: {d31:.2f}"
            )

        iteration += 1

        # Maintain controller frequency
        elapsed = time.time() - loop_start

        time.sleep(max(0, DT - elapsed))

    print("\nFormation experiment completed.")


# ============================================================
# 10. LANDING
# ============================================================

def land_drones(connections):

    print("\nLanding drones...")

    for scf in connections:

        scf.cf.high_level_commander.land(
            0.0,
            2.0
        )

    time.sleep(3)

    for scf in connections:

        scf.cf.commander.send_stop_setpoint()

    print("Landing complete.")


# ============================================================
# 11. MAIN
# ============================================================

def main():

    connections = []

    log_configs = []

    try:

        print("======================================")
        print("LAGRANGIAN THREE-DRONE FORMATION")
        print("======================================")

        # Connect all three drones
        connections, log_configs = connect_drones()

        # Wait for position and velocity estimates
        wait_for_telemetry()

        # Takeoff
        takeoff_drones(connections)

        # Display initial positions
        with state_lock:
            initial_positions = positions[:, :2].copy()

        print("\nInitial positions:")

        for i in range(N):
            print(f"Drone {i+1}: {initial_positions[i]}")

        print("\nTarget:", target)

        # Run formation controller
        run_formation(connections)

    except KeyboardInterrupt:

        print("\nExperiment interrupted.")

    except Exception as error:

        print(f"\nExperiment error: {error}")

        raise

    finally:

        # Attempt safe landing
        if connections:

            try:
                land_drones(connections)

            except Exception as error:
                print("Landing warning:", error)

        # Stop telemetry
        for logconf in log_configs:

            try:
                logconf.stop()

            except Exception:
                pass

        # Close connections
        for scf in connections:

            try:
                scf.close_link()

            except Exception:
                pass

        print("\nAll drone connections closed.")


if __name__ == "__main__":

    main()
