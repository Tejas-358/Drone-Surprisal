
# # """
# # Surprisal-adaptive Euler-Lagrange-inspired distance control in CrazySim.

# # Run from the Drone-Surprisal repository root:
# #     python -m src.tests.surprisal_distance_crazysim --mode sim

# # Requirements:
# # - CrazySim/Gazebo is running with two Crazyflies.
# # - The project configuration contains two SIMULATION URIs.
# # - Existing fly_safely() manages connection, takeoff and landing.

# # The disturbance is a brief velocity-setpoint pulse, not a physical
# # force applied inside Gazebo. Energies are estimates from telemetry.
# # """

# # import argparse
# # import csv
# # import math
# # import time
# # from pathlib import Path

# # from src.utils.connection import load_config
# # from src.utils.flight import fly_safely
# # from src.utils.safety import watch

# # import warnings

# # warnings.filterwarnings("ignore")


# # # ============================================================
# # # EXPERIMENT SETTINGS
# # # ============================================================

# # DT = 0.1
# # DURATION = 30.0

# # DESIRED_DISTANCE = 0.40
# # DISTANCE_TOLERANCE = 0.08
# # CRITICAL_DISTANCE = 0.20

# # # Velocity-level controller.
# # BASE_KP = 0.20
# # ADAPTIVE_GAIN = 0.50
# # MAX_KP = 0.60
# # MAX_SPEED = 0.08
# # CENTER_HOLD_KP = 0.10
# # ALTITUDE_KP = 0.30

# # # Surprisal model.
# # SURPRISE_SIGMA = 0.15

# # # Approximate effective masses, for energy estimates only.
# # MASS_1 = 0.035
# # MASS_2 = 0.035

# # # Simulated disturbance: briefly command CF1 toward CF2.
# # DISTURBANCE_TIME = 5.0
# # DISTURBANCE_DURATION = 0.25
# # DISTURBANCE_SPEED = 0.12

# # # Basic telemetry/world bounds.
# # MIN_Z = 0.15
# # MAX_Z = 0.70
# # MAX_ABS_XY = 3.0

# # OUTPUT_DIR = Path("data/raw")
# # LOG_FILE = OUTPUT_DIR / "surprisal_distance_crazysim.csv"


# # # ============================================================
# # # HELPERS
# # # ============================================================

# # def read_position(state):
# #     try:
# #         p = (
# #             float(state["x"]),
# #             float(state["y"]),
# #             float(state["z"]),
# #         )
# #     except (KeyError, TypeError, ValueError) as exc:
# #         raise RuntimeError(
# #             "Missing or invalid simulated position telemetry."
# #         ) from exc

# #     if not all(math.isfinite(v) for v in p):
# #         raise RuntimeError(f"Invalid position telemetry: {p}")

# #     return p


# # def distance_xy(p1, p2):
# #     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


# # def clip_velocity(vx, vy, vz):
# #     speed = math.sqrt(vx * vx + vy * vy + vz * vz)

# #     if speed > MAX_SPEED:
# #         scale = MAX_SPEED / speed
# #         vx *= scale
# #         vy *= scale
# #         vz *= scale

# #     return vx, vy, vz


# # def send_velocity(drone, velocity):
# #     vx, vy, vz = velocity
# #     drone.cf.commander.send_velocity_world_setpoint(
# #         vx, vy, vz, 0.0
# #     )


# # def check_flight_envelope(position, name):
# #     x, y, z = position

# #     if abs(x) > MAX_ABS_XY or abs(y) > MAX_ABS_XY:
# #         raise RuntimeError(
# #             f"{name} exceeded the configured XY envelope: {position}"
# #         )

# #     if not MIN_Z <= z <= MAX_Z:
# #         raise RuntimeError(
# #             f"{name} altitude outside "
# #             f"[{MIN_Z:.2f}, {MAX_Z:.2f}] m: {position}"
# #         )


# # def calculate_surprisal(distance):
# #     error = distance - DESIRED_DISTANCE
# #     return 0.5 * (error / SURPRISE_SIGMA) ** 2


# # def calculate_commands(positions, anchors):
# #     """
# #     Surprisal-adaptive distance controller.

# #     A positive DESIRED_DISTANCE - actual_distance means the drones
# #     need to move apart. A negative value means they need to move
# #     gently closer together.

# #     Returns:
# #         distance, error, surprisal, stiffness, velocities
# #     """
# #     p1, p2 = positions
# #     a1, a2 = anchors

# #     dx = p2[0] - p1[0]
# #     dy = p2[1] - p1[1]
# #     d = math.hypot(dx, dy)

# #     if d < 1e-6:
# #         raise RuntimeError(
# #             "Relative position is undefined; aborting the experiment."
# #         )

# #     ux = dx / d
# #     uy = dy / d

# #     # Positive means the drones are too close.
# #     error = DESIRED_DISTANCE - d
# #     surprisal = calculate_surprisal(d)

# #     # Adaptive feedback gain, bounded to avoid unlimited amplification.
# #     kp = min(
# #         BASE_KP * (1.0 + ADAPTIVE_GAIN * surprisal),
# #         MAX_KP,
# #     )

# #     correction = kp * error

# #     # Weakly keep the formation's centre near its starting point.
# #     center_x = 0.5 * (p1[0] + p2[0])
# #     center_y = 0.5 * (p1[1] + p2[1])

# #     anchor_center_x = 0.5 * (a1[0] + a2[0])
# #     anchor_center_y = 0.5 * (a1[1] + a2[1])

# #     center_vx = CENTER_HOLD_KP * (
# #         anchor_center_x - center_x
# #     )
# #     center_vy = CENTER_HOLD_KP * (
# #         anchor_center_y - center_y
# #     )

# #     # Symmetric distance correction.
# #     v1x = center_vx - correction * ux
# #     v1y = center_vy - correction * uy

# #     v2x = center_vx + correction * ux
# #     v2y = center_vy + correction * uy

# #     # Hold each drone near its own initial altitude.
# #     v1z = ALTITUDE_KP * (a1[2] - p1[2])
# #     v2z = ALTITUDE_KP * (a2[2] - p2[2])

# #     velocities = [
# #         clip_velocity(v1x, v1y, v1z),
# #         clip_velocity(v2x, v2y, v2z),
# #     ]

# #     stiffness = kp

# #     return d, error, surprisal, stiffness, velocities


# # # ============================================================
# # # EXPERIMENT
# # # ============================================================

# # def mission(drones, config, states):
# #     if len(drones) != 2 or len(states) != 2:
# #         raise RuntimeError("Exactly two simulated drones are required.")

# #     names = ["CF1", "CF2"]

# #     positions = [read_position(s) for s in states]
# #     anchors = list(positions)

# #     for i, p in enumerate(positions):
# #         check_flight_envelope(p, names[i])

# #     initial_distance = distance_xy(positions[0], positions[1])

# #     print("\n" + "=" * 60)
# #     print("SURPRISAL-ADAPTIVE DISTANCE MAINTENANCE")
# #     print("=" * 60)
# #     print(f"CF1 start: {positions[0]}")
# #     print(f"CF2 start: {positions[1]}")
# #     print(f"Initial distance: {initial_distance:.3f} m")
# #     print(f"Desired distance: {DESIRED_DISTANCE:.3f} m")
# #     print(f"Disturbance scheduled at: {DISTURBANCE_TIME:.1f} s")

# #     if initial_distance <= CRITICAL_DISTANCE:
# #         raise RuntimeError(
# #             "Initial separation is too small for this experiment. "
# #             "Stop and reposition the simulated drones before retrying."
# #         )

# #     OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# #     fields = [
# #         "time_s",
# #         "x1_m", "y1_m", "z1_m",
# #         "x2_m", "y2_m", "z2_m",
# #         "distance_m", "distance_error_m",
# #         "surprisal", "adaptive_gain",
# #         "kinetic_energy_J",
# #         "potential_energy_J",
# #         "mechanical_energy_J",
# #         "vx1_cmd_m_s", "vy1_cmd_m_s", "vz1_cmd_m_s",
# #         "vx2_cmd_m_s", "vy2_cmd_m_s", "vz2_cmd_m_s",
# #         "disturbance_active",
# #     ]

# #     start_time = time.monotonic()
# #     last_log = start_time
# #     previous_positions = positions
# #     previous_time = start_time
# #     disturbance_end = None
# #     disturbance_started = False

# #     with LOG_FILE.open("w", newline="", encoding="utf-8") as f:
# #         writer = csv.DictWriter(f, fieldnames=fields)
# #         writer.writeheader()

# #         while True:
# #             now = time.monotonic()
# #             elapsed = now - start_time

# #             positions = [read_position(s) for s in states]

# #             for i, p in enumerate(positions):
# #                 check_flight_envelope(p, names[i])

# #             d, error, surprise, kp, velocities = calculate_commands(
# #                 positions, anchors
# #             )

# #             if d <= CRITICAL_DISTANCE:
# #                 # Stop issuing normal navigation commands and abort.
# #                 # fly_safely() should handle its configured cleanup.
# #                 raise RuntimeError(
# #                     f"Separation reached critical threshold: {d:.3f} m. "
# #                     "Aborting distance-control experiment."
# #                 )

# #             # A brief simulated perturbation: command CF1 toward CF2.
# #             # After this pulse, normal distance control resumes.
# #             disturbance_active = (
# #                 DISTURBANCE_TIME <= elapsed
# #                 < DISTURBANCE_TIME + DISTURBANCE_DURATION
# #             )

# #             if (
# #                 elapsed >= DISTURBANCE_TIME
# #                 and not disturbance_started
# #             ):
# #                 print("\n>>> Simulated disturbance started")
# #                 disturbance_started = True

# #             if (
# #                 disturbance_started
# #                 and disturbance_end is None
# #                 and elapsed >= DISTURBANCE_TIME + DISTURBANCE_DURATION
# #             ):
# #                 disturbance_end = elapsed
# #                 print(">>> Disturbance ended; controller recovering\n")

# #             if disturbance_active:
# #                 dx = positions[1][0] - positions[0][0]
# #                 dy = positions[1][1] - positions[0][1]
# #                 norm = max(math.hypot(dx, dy), 1e-9)

# #                 pulse = (
# #                     DISTURBANCE_SPEED * dx / norm,
# #                     DISTURBANCE_SPEED * dy / norm,
# #                     0.0,
# #                 )

# #                 # Drone 1 receives the pulse; drone 2 temporarily holds.
# #                 commands = [
# #                     pulse,
# #                     (0.0, 0.0, 0.0),
# #                 ]
# #             else:
# #                 commands = velocities

# #             # Estimate velocities from successive position samples.
# #             dt_measured = max(now - previous_time, 1e-6)

# #             estimated_v = []
# #             for i in range(2):
# #                 estimated_v.append(tuple(
# #                     (positions[i][k] - previous_positions[i][k])
# #                     / dt_measured
# #                     for k in range(3)
# #                 ))

# #             kinetic_energy = (
# #                 0.5 * MASS_1 * sum(v * v for v in estimated_v[0])
# #                 + 0.5 * MASS_2 * sum(v * v for v in estimated_v[1])
# #             )

# #             # This is an effective virtual potential, not physical spring energy.
# #             potential_energy = 0.5 * kp * error * error
# #             mechanical_energy = kinetic_energy + potential_energy

# #             writer.writerow({
# #                 "time_s": elapsed,
# #                 "x1_m": positions[0][0],
# #                 "y1_m": positions[0][1],
# #                 "z1_m": positions[0][2],
# #                 "x2_m": positions[1][0],
# #                 "y2_m": positions[1][1],
# #                 "z2_m": positions[1][2],
# #                 "distance_m": d,
# #                 "distance_error_m": -error,
# #                 "surprisal": surprise,
# #                 "adaptive_gain": kp,
# #                 "kinetic_energy_J": kinetic_energy,
# #                 "potential_energy_J": potential_energy,
# #                 "mechanical_energy_J": mechanical_energy,
# #                 "vx1_cmd_m_s": commands[0][0],
# #                 "vy1_cmd_m_s": commands[0][1],
# #                 "vz1_cmd_m_s": commands[0][2],
# #                 "vx2_cmd_m_s": commands[1][0],
# #                 "vy2_cmd_m_s": commands[1][1],
# #                 "vz2_cmd_m_s": commands[1][2],
# #                 "disturbance_active": disturbance_active,
# #             })

# #             # Flush so the log remains useful if the experiment aborts.
# #             f.flush()

# #             # Commands are setpoints, not direct physical forces.
# #             for i in range(2):
# #                 send_velocity(drones[i], commands[i])

# #             if now - last_log >= 0.5:
# #                 print(
# #                     f"t={elapsed:5.1f}s | "
# #                     f"d={d:.3f}m | "
# #                     f"error={-error:+.3f}m | "
# #                     f"S={surprise:.3f} | "
# #                     f"K={kinetic_energy:.5f}J | "
# #                     f"V={potential_energy:.5f}J"
# #                 )
# #                 last_log = now

# #             if elapsed >= DURATION:
# #                 break

# #             previous_positions = positions
# #             previous_time = now

# #             watch(states, config, DT)

# #     # Request zero velocity before fly_safely() begins landing.
# #     for drone in drones:
# #         send_velocity(drone, (0.0, 0.0, 0.0))

# #     print(f"\nExperiment complete. Log saved to: {LOG_FILE.resolve()}")
# #     print("Returning to fly_safely() for landing.")


# # def main():
# #     parser = argparse.ArgumentParser()
# #     parser.add_argument(
# #         "--mode",
# #         choices=["sim", "real"],
# #         default="sim",
# #         help="Use sim for CrazySim.",
# #     )
# #     args = parser.parse_args()

# #     if args.mode != "sim":
# #         raise RuntimeError(
# #             "This experiment script is intended for simulation only."
# #         )

# #     config = load_config()
# #     config["mode"] = "sim"

# #     fly_safely(config, 2, mission)


# # if __name__ == "__main__":
# #     main()


# """
# surprisal_distance_crazysim.py

# Two-Crazyflie CrazySim experiment:
# - Maintain a desired inter-drone distance.
# - Increase proportional correction as distance surprisal increases.
# - Use relative-velocity damping to improve recovery.
# - Apply a short simulated velocity disturbance to CF1.
# - Log position, distance, surprisal, controller response, and energy estimates.

# Run from the Drone-Surprisal repository root:
#     python -m src.tests.surprisal_distance_crazysim --mode sim
# """

# import argparse
# import csv
# import math
# import time
# from pathlib import Path

# import numpy as np

# from src.utils.connection import load_config
# from src.utils.flight import fly_safely
# from src.utils.safety import watch

# import warnings
# warnings.filterwarnings("ignore")


# # ============================================================
# # EXPERIMENT SETTINGS
# # ============================================================

# DT = 0.1
# DURATION = 30.0

# DESIRED_DISTANCE = 0.90
# DISTANCE_TOLERANCE = 0.08
# CRITICAL_DISTANCE = 0.50

# ALTITUDE_TARGET = 0.40
# ALTITUDE_KP = 0.8
# MAX_VERTICAL_SPEED = 0.15

# # Surprisal-adaptive distance controller
# BASE_KP = 0.65
# SURPRISE_GAIN = 2.0
# MAX_KP = 1.50
# SIGMA_DISTANCE = 0.08

# # Relative-velocity damping
# KD_RELATIVE = 0.45

# # Per-drone horizontal speed limit
# MAX_HORIZONTAL_SPEED = 0.20

# # Keep the experiment inside a bounded simulation area
# MAX_ABS_XY = 2.5
# MIN_ALTITUDE = 0.20
# MAX_ALTITUDE = 0.80

# # Disturbance: a short velocity pulse applied to CF1
# DISTURBANCE_START = 5.0
# DISTURBANCE_DURATION = 0.25
# DISTURBANCE_SPEED = 0.12

# # Approximate Crazyflie mass, used only for kinetic-energy estimates
# DRONE_MASS_KG = 0.035

# OUTPUT_CSV = Path("data/raw/surprisal_distance_crazysim.csv")


# # ============================================================
# # HELPER FUNCTIONS
# # ============================================================

# def limit_vector(vector, max_norm):
#     """Limit a vector's magnitude without changing its direction."""
#     vector = np.asarray(vector, dtype=float)
#     norm = float(np.linalg.norm(vector))

#     if norm > max_norm and norm > 1e-12:
#         return vector * (max_norm / norm)

#     return vector


# def read_position(state):
#     """Read world-frame XYZ position from the shared telemetry state."""
#     return np.array(
#         [float(state["x"]), float(state["y"]), float(state["z"])],
#         dtype=float,
#     )


# def send_velocity(scf, vx, vy, vz):
#     """Send a world-frame velocity setpoint."""
#     scf.cf.commander.send_velocity_world_setpoint(
#         float(vx),
#         float(vy),
#         float(vz),
#         0.0,
#     )


# def check_envelope(p1, p2, distance):
#     """Stop the mission if the simulated flight leaves its safety envelope."""
#     if distance < CRITICAL_DISTANCE:
#         raise RuntimeError(
#             f"Critical separation reached: {distance:.3f} m "
#             f"(minimum configured separation: {CRITICAL_DISTANCE:.3f} m)"
#         )

#     for drone_name, p in (("CF1", p1), ("CF2", p2)):
#         if abs(p[0]) > MAX_ABS_XY or abs(p[1]) > MAX_ABS_XY:
#             raise RuntimeError(
#                 f"{drone_name} exceeded the XY simulation envelope: "
#                 f"x={p[0]:.2f}, y={p[1]:.2f}"
#             )

#         if not MIN_ALTITUDE <= p[2] <= MAX_ALTITUDE:
#             raise RuntimeError(
#                 f"{drone_name} left the altitude envelope: z={p[2]:.2f} m"
#             )


# def prepare_log():
#     OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)

#     fieldnames = [
#         "time_s",
#         "cf1_x", "cf1_y", "cf1_z",
#         "cf2_x", "cf2_y", "cf2_z",
#         "distance_m",
#         "distance_error_m",
#         "distance_within_tolerance",
#         "surprisal",
#         "kp_adaptive",
#         "radial_velocity_m_s",
#         "distance_command_m_s",
#         "cf1_vx_command", "cf1_vy_command", "cf1_vz_command",
#         "cf2_vx_command", "cf2_vy_command", "cf2_vz_command",
#         "estimated_cf1_speed_m_s",
#         "estimated_cf2_speed_m_s",
#         "kinetic_energy_estimate_J",
#         "virtual_distance_potential_proxy",
#         "mechanical_energy_proxy",
#         "disturbance_active",
#     ]

#     file_handle = OUTPUT_CSV.open("w", newline="", encoding="utf-8")
#     writer = csv.DictWriter(file_handle, fieldnames=fieldnames)
#     writer.writeheader()

#     return file_handle, writer


# # ============================================================
# # MAIN TWO-DRONE MISSION
# # ============================================================

# def mission(drones, config, states):
#     if len(drones) != 2 or len(states) != 2:
#         raise RuntimeError(
#             "This experiment requires exactly two connected Crazyflies "
#             "and two telemetry states."
#         )

#     cf1, cf2 = drones[0], drones[1]

#     print("\n" + "=" * 60)
#     print("SURPRISAL-ADAPTIVE DISTANCE MAINTENANCE")
#     print("=" * 60)

#     # Wait for telemetry to settle after takeoff.
#     watch(states, config, 1.0)

#     p1 = read_position(states[0])
#     p2 = read_position(states[1])

#     initial_distance = float(np.linalg.norm(p2[:2] - p1[:2]))

#     print(f"CF1 start: {tuple(p1)}")
#     print(f"CF2 start: {tuple(p2)}")
#     print(f"Initial distance: {initial_distance:.3f} m")
#     print(f"Desired distance: {DESIRED_DISTANCE:.3f} m")
#     print(f"Critical distance: {CRITICAL_DISTANCE:.3f} m")
#     print(f"Disturbance scheduled at: {DISTURBANCE_START:.1f} s")

#     if initial_distance <= CRITICAL_DISTANCE:
#         raise RuntimeError(
#             "Initial separation is at or below the critical distance. "
#             "Aborting the experiment."
#         )

#     # Store previous telemetry to estimate velocity.
#     previous_p1 = p1.copy()
#     previous_p2 = p2.copy()
#     previous_time = time.monotonic()

#     log_file, logger = prepare_log()

#     start_time = time.monotonic()
#     next_log_time = 0.0
#     disturbance_announced = False
#     disturbance_end_announced = False

#     try:
#         while True:
#             elapsed = time.monotonic() - start_time

#             if elapsed >= DURATION:
#                 break

#             # Let the project's safety watcher update/check shared states.
#             watch(states, config, DT)

#             now = time.monotonic()
#             sample_dt = max(now - previous_time, 1e-3)

#             p1 = read_position(states[0])
#             p2 = read_position(states[1])

#             # Estimate world-frame velocities from position differences.
#             v1 = (p1 - previous_p1) / sample_dt
#             v2 = (p2 - previous_p2) / sample_dt

#             previous_p1 = p1.copy()
#             previous_p2 = p2.copy()
#             previous_time = now

#             # Distance is measured in the horizontal plane.
#             relative_position = p2[:2] - p1[:2]
#             distance = float(np.linalg.norm(relative_position))

#             check_envelope(p1, p2, distance)

#             if distance < 1e-8:
#                 raise RuntimeError(
#                     "Drone separation is too small to calculate a direction."
#                 )

#             direction = relative_position / distance
#             distance_error = distance - DESIRED_DISTANCE

#             # Gaussian distance surprisal.
#             surprisal = 0.5 * (
#                 distance_error / SIGMA_DISTANCE
#             ) ** 2

#             # Bounded surprisal-adaptive proportional gain.
#             kp_adaptive = min(
#                 BASE_KP * (1.0 + SURPRISE_GAIN * surprisal),
#                 MAX_KP,
#             )

#             # Relative radial velocity:
#             # positive = drones are moving apart;
#             # negative = drones are approaching each other.
#             relative_velocity = v2[:2] - v1[:2]
#             radial_velocity = float(
#                 np.dot(relative_velocity, direction)
#             )

#             # Positive command means both drones should move toward
#             # one another; negative means they should move apart.
#             distance_command = (
#                 kp_adaptive * distance_error
#                 + KD_RELATIVE * radial_velocity
#             )

#             # Symmetric commands help keep the pair's center stable.
#             correction = distance_command * direction

#             cf1_xy = correction.copy()
#             cf2_xy = -correction.copy()

#             # Add a bounded, short setpoint disturbance to CF1.
#             disturbance_active = (
#                 DISTURBANCE_START
#                 <= elapsed
#                 < DISTURBANCE_START + DISTURBANCE_DURATION
#             )

#             if disturbance_active:
#                 cf1_xy += DISTURBANCE_SPEED * direction

#                 if not disturbance_announced:
#                     print("\n>>> Simulated disturbance started")
#                     disturbance_announced = True

#             elif (
#                 elapsed >= DISTURBANCE_START + DISTURBANCE_DURATION
#                 and disturbance_announced
#                 and not disturbance_end_announced
#             ):
#                 print(">>> Disturbance ended; controller recovering\n")
#                 disturbance_end_announced = True

#             # Bound the horizontal commands after adding the disturbance.
#             cf1_xy = limit_vector(cf1_xy, MAX_HORIZONTAL_SPEED)
#             cf2_xy = limit_vector(cf2_xy, MAX_HORIZONTAL_SPEED)

#             # Altitude hold around the takeoff altitude.
#             vz1 = float(np.clip(
#                 ALTITUDE_KP * (ALTITUDE_TARGET - p1[2]),
#                 -MAX_VERTICAL_SPEED,
#                 MAX_VERTICAL_SPEED,
#             ))
#             vz2 = float(np.clip(
#                 ALTITUDE_KP * (ALTITUDE_TARGET - p2[2]),
#                 -MAX_VERTICAL_SPEED,
#                 MAX_VERTICAL_SPEED,
#             ))

#             # Send commands to the simulated Crazyflies.
#             send_velocity(cf1, cf1_xy[0], cf1_xy[1], vz1)
#             send_velocity(cf2, cf2_xy[0], cf2_xy[1], vz2)

#             # ----------------------------------------------------
#             # Energy estimates
#             # ----------------------------------------------------
#             # These are telemetry-based estimates, not direct measurements.
#             speed1 = float(np.linalg.norm(v1))
#             speed2 = float(np.linalg.norm(v2))

#             kinetic_energy = 0.5 * DRONE_MASS_KG * (
#                 speed1**2 + speed2**2
#             )

#             # This is a virtual potential proxy for the distance error.
#             # The adaptive controller gain has velocity-control units,
#             # so this is NOT a physically calibrated energy in joules.
#             potential_proxy = 0.5 * kp_adaptive * distance_error**2
#             mechanical_proxy = kinetic_energy + potential_proxy

#             # ----------------------------------------------------
#             # Logging
#             # ----------------------------------------------------
#             row = {
#                 "time_s": round(elapsed, 3),
#                 "cf1_x": p1[0],
#                 "cf1_y": p1[1],
#                 "cf1_z": p1[2],
#                 "cf2_x": p2[0],
#                 "cf2_y": p2[1],
#                 "cf2_z": p2[2],
#                 "distance_m": distance,
#                 "distance_error_m": distance_error,
#                 "distance_within_tolerance": (
#                     abs(distance_error) <= DISTANCE_TOLERANCE
#                 ),
#                 "surprisal": surprisal,
#                 "kp_adaptive": kp_adaptive,
#                 "radial_velocity_m_s": radial_velocity,
#                 "distance_command_m_s": distance_command,
#                 "cf1_vx_command": cf1_xy[0],
#                 "cf1_vy_command": cf1_xy[1],
#                 "cf1_vz_command": vz1,
#                 "cf2_vx_command": cf2_xy[0],
#                 "cf2_vy_command": cf2_xy[1],
#                 "cf2_vz_command": vz2,
#                 "estimated_cf1_speed_m_s": speed1,
#                 "estimated_cf2_speed_m_s": speed2,
#                 "kinetic_energy_estimate_J": kinetic_energy,
#                 "virtual_distance_potential_proxy": potential_proxy,
#                 "mechanical_energy_proxy": mechanical_proxy,
#                 "disturbance_active": disturbance_active,
#             }
#             logger.writerow(row)
#             log_file.flush()

#             if elapsed >= next_log_time:
#                 print(
#                     f"t={elapsed:5.1f}s | "
#                     f"d={distance:.3f}m | "
#                     f"error={distance_error:+.3f}m | "
#                     f"S={surprisal:.3f} | "
#                     f"Kp={kp_adaptive:.3f} | "
#                     f"KE~{kinetic_energy:.6f}J | "
#                     f"Vproxy={potential_proxy:.6f}"
#                 )
#                 next_log_time += 0.5

#     finally:
#         log_file.close()

#         # Send zero horizontal velocity commands before returning.
#         # fly_safely should handle the project's landing/cleanup sequence.
#         try:
#             send_velocity(cf1, 0.0, 0.0, 0.0)
#             send_velocity(cf2, 0.0, 0.0, 0.0)
#         except Exception:
#             pass

#     print("\nExperiment complete.")
#     print(f"CSV saved to: {OUTPUT_CSV.resolve()}")
#     return str(OUTPUT_CSV)


# # ============================================================
# # ENTRY POINT
# # ============================================================

# def main():
#     parser = argparse.ArgumentParser(
#         description="Run the two-drone surprisal distance experiment."
#     )
#     parser.add_argument(
#         "--mode",
#         choices=["sim"],
#         default="sim",
#         help="This test is restricted to simulation mode.",
#     )
#     args = parser.parse_args()

#     if args.mode != "sim":
#         raise RuntimeError("This experiment is intended for CrazySim only.")

#     config = load_config()

#     # Ensure the loaded configuration actually targets the simulator.
#     # The project's config loader should select UDP/SITL URIs for sim mode.
#     fly_safely(config, 2, mission)


# if __name__ == "__main__":
#     main()



"""
surprisal_goal_navigation_real.py

Real Crazyflie experiment:
- Two drones move toward a shared formation-center goal.
- A surprisal-adaptive controller maintains their desired separation.
- Relative-velocity damping helps reduce overshoot.
- A protective separation response overrides goal navigation when needed.
- Logs navigation, separation, surprisal, and velocity commands.

Run from the Drone-Surprisal repository root:
    python -m src.tests.surprisal_goal_navigation_real --mode real

IMPORTANT:
This is an experimental controller, NOT a certified collision-avoidance
system. Validate in CrazySim first. Do not physically push flying drones.
"""

import argparse
import csv
import time
from pathlib import Path

import numpy as np

from src.utils.connection import load_config
from src.utils.flight import fly_safely
from src.utils.safety import watch

import warnings

warnings.filterwarnings("ignore")


# ============================================================
# NAVIGATION SETTINGS
# ============================================================

DT = 0.1
DURATION = 25.0

# Formation center will move this far from its initial position.
# Example: (0.75, 0.0) means move 0.75 m along world +X.
GOAL_OFFSET_XY = np.array([1.00, 0.30], dtype=float)

GOAL_TOLERANCE = 0.08
GOAL_KP = 0.20
MAX_GOAL_SPEED = 0.08

# Keep the formation centered around its planned path.
CENTER_KD = 0.15

# ============================================================
# SEPARATION AND SURPRISAL SETTINGS
# ============================================================

DESIRED_DISTANCE = 0.90
DISTANCE_TOLERANCE = 0.08

# Protective response activates before the abort threshold.
PROTECTIVE_DISTANCE = 0.75
CRITICAL_DISTANCE = 0.60

BASE_KP = 0.25
SURPRISE_GAIN = 1.5
MAX_KP = 0.45
SIGMA_DISTANCE = 0.10

KD_RELATIVE = 0.25
SEPARATION_RECOVERY_GAIN = 0.50

# ============================================================
# FLIGHT LIMITS
# ============================================================

TARGET_ALTITUDE = 0.40
ALTITUDE_KP = 0.50
MAX_VERTICAL_SPEED = 0.08

MAX_HORIZONTAL_SPEED = 0.10
MAX_POSITION_DEVIATION = 1.20

MIN_ALTITUDE = 0.20
MAX_ALTITUDE = 0.80

VELOCITY_FILTER = 0.35

OUTPUT_CSV = Path(
    "data/raw/surprisal_goal_navigation_real.csv"
)


# ============================================================
# HELPERS
# ============================================================

def limit_vector(vector, max_norm):
    """Limit a vector's magnitude."""
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))

    if norm > max_norm and norm > 1e-9:
        return vector * (max_norm / norm)

    return vector


def read_position(state):
    """Read world-frame XYZ position from Lighthouse telemetry."""
    return np.array(
        [
            float(state["x"]),
            float(state["y"]),
            float(state["z"]),
        ],
        dtype=float,
    )


def send_velocity(scf, vx, vy, vz):
    """Send world-frame velocity commands."""
    scf.cf.commander.send_velocity_world_setpoint(
        float(vx),
        float(vy),
        float(vz),
        0.0,
    )


def stop_motion(drones):
    """Best-effort zero-velocity command."""
    for drone in drones:
        try:
            send_velocity(drone, 0.0, 0.0, 0.0)
        except Exception:
            pass


def check_safety(p1, p2, start1, start2, distance):
    """Check the configured flight envelope."""
    if distance <= CRITICAL_DISTANCE:
        raise RuntimeError(
            f"CRITICAL SEPARATION: {distance:.3f} m. "
            "Aborting mission."
        )

    for name, position, initial in (
        ("CF1", p1, start1),
        ("CF2", p2, start2),
    ):
        if not MIN_ALTITUDE <= position[2] <= MAX_ALTITUDE:
            raise RuntimeError(
                f"{name} altitude outside configured limits: "
                f"{position[2]:.3f} m"
            )

        displacement = float(
            np.linalg.norm(position[:2] - initial[:2])
        )

        if displacement > MAX_POSITION_DEVIATION:
            raise RuntimeError(
                f"{name} exceeded XY displacement limit: "
                f"{displacement:.3f} m"
            )


# ============================================================
# MISSION
# ============================================================

def mission(drones, config, states):
    if len(drones) != 2 or len(states) != 2:
        raise RuntimeError(
            "This experiment requires exactly two Crazyflies."
        )

    cf1, cf2 = drones

    print("\n" + "=" * 65)
    print("SURPRISAL-ADAPTIVE GOAL NAVIGATION")
    print("=" * 65)
    print("Hardware: REAL Crazyflies")
    print("Position source: Lighthouse/world telemetry")
    print("Goal: translate the formation while maintaining separation")
    print("No artificial disturbance will be injected.")

    # Allow telemetry to settle after the existing takeoff sequence.
    watch(states, config, 1.0)

    p1 = read_position(states[0])
    p2 = read_position(states[1])

    start1 = p1.copy()
    start2 = p2.copy()

    initial_center = 0.5 * (p1[:2] + p2[:2])
    goal_xy = initial_center + GOAL_OFFSET_XY

    distance = float(np.linalg.norm(p2[:2] - p1[:2]))

    print(f"CF1 initial position: {p1}")
    print(f"CF2 initial position: {p2}")
    print(f"Initial separation:   {distance:.3f} m")
    print(f"Desired separation:   {DESIRED_DISTANCE:.3f} m")
    print(f"Formation goal XY:    {goal_xy}")
    print(f"Protective distance:  {PROTECTIVE_DISTANCE:.3f} m")
    print(f"Critical distance:    {CRITICAL_DISTANCE:.3f} m")

    if distance <= PROTECTIVE_DISTANCE:
        raise RuntimeError(
            "Initial separation is too small for this experiment. "
            "Reposition the drones on the ground."
        )

    previous_p1 = p1.copy()
    previous_p2 = p2.copy()
    previous_time = time.monotonic()

    v1 = np.zeros(3, dtype=float)
    v2 = np.zeros(3, dtype=float)

    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)

    fields = [
        "time_s",
        "cf1_x", "cf1_y", "cf1_z",
        "cf2_x", "cf2_y", "cf2_z",
        "goal_x", "goal_y",
        "center_x", "center_y",
        "goal_error_m",
        "distance_m", "distance_error_m",
        "surprisal", "adaptive_kp",
        "radial_velocity_m_s",
        "protective_mode",
        "cf1_vx_cmd", "cf1_vy_cmd", "cf1_vz_cmd",
        "cf2_vx_cmd", "cf2_vy_cmd", "cf2_vz_cmd",
        "cf1_speed_est_m_s", "cf2_speed_est_m_s",
    ]

    start_time = time.monotonic()
    next_print = 0.0

    try:
        with OUTPUT_CSV.open(
            "w", newline="", encoding="utf-8"
        ) as file:
            logger = csv.DictWriter(file, fieldnames=fields)
            logger.writeheader()

            while time.monotonic() - start_time < DURATION:
                watch(states, config, DT)

                now = time.monotonic()
                elapsed = now - start_time
                sample_dt = max(now - previous_time, 1e-3)

                p1 = read_position(states[0])
                p2 = read_position(states[1])

                # Estimate velocities from position telemetry and filter noise.
                raw_v1 = (p1 - previous_p1) / sample_dt
                raw_v2 = (p2 - previous_p2) / sample_dt

                v1 = (
                    VELOCITY_FILTER * raw_v1
                    + (1.0 - VELOCITY_FILTER) * v1
                )
                v2 = (
                    VELOCITY_FILTER * raw_v2
                    + (1.0 - VELOCITY_FILTER) * v2
                )

                previous_p1 = p1.copy()
                previous_p2 = p2.copy()
                previous_time = now

                relative_position = p2[:2] - p1[:2]
                distance = float(np.linalg.norm(relative_position))

                check_safety(
                    p1, p2, start1, start2, distance
                )

                if distance < 1e-6:
                    raise RuntimeError(
                        "Unable to calculate relative direction."
                    )

                direction = relative_position / distance
                distance_error = distance - DESIRED_DISTANCE

                # Distance surprisal.
                surprisal = 0.5 * (
                    distance_error / SIGMA_DISTANCE
                ) ** 2

                adaptive_kp = min(
                    BASE_KP * (
                        1.0 + SURPRISE_GAIN * surprisal
                    ),
                    MAX_KP,
                )

                relative_velocity = v2[:2] - v1[:2]
                radial_velocity = float(
                    np.dot(relative_velocity, direction)
                )

                # Positive command pulls the drones closer.
                # Negative command pushes them apart.
                distance_command = (
                    adaptive_kp * distance_error
                    + KD_RELATIVE * radial_velocity
                )

                protective_mode = (
                    distance < PROTECTIVE_DISTANCE
                )

                if protective_mode:
                    # Override normal formation control with an
                    # outward separation response.
                    safety_command = -(
                        SEPARATION_RECOVERY_GAIN
                        * (PROTECTIVE_DISTANCE - distance)
                        + KD_RELATIVE * max(0.0, -radial_velocity)
                    )

                    distance_command = min(
                        distance_command,
                        safety_command,
                    )

                # ------------------------------------------------
                # GOAL NAVIGATION
                # ------------------------------------------------
                center_xy = 0.5 * (p1[:2] + p2[:2])
                goal_error = goal_xy - center_xy
                goal_error_norm = float(np.linalg.norm(goal_error))

                if goal_error_norm <= GOAL_TOLERANCE:
                    goal_velocity = np.zeros(2, dtype=float)
                else:
                    goal_velocity = limit_vector(
                        GOAL_KP * goal_error,
                        MAX_GOAL_SPEED,
                    )

                # Reduce shared goal movement if separation protection
                # is active; prioritize creating separation.
                if protective_mode:
                    goal_velocity *= 0.25

                # ------------------------------------------------
                # COMBINE GOAL AND DISTANCE CONTROL
                # ------------------------------------------------
                correction = 0.5 * distance_command * direction

                cf1_xy = goal_velocity + correction
                cf2_xy = goal_velocity - correction

                # Bound commands independently.
                cf1_xy = limit_vector(
                    cf1_xy, MAX_HORIZONTAL_SPEED
                )
                cf2_xy = limit_vector(
                    cf2_xy, MAX_HORIZONTAL_SPEED
                )

                # Altitude hold.
                vz1 = float(np.clip(
                    ALTITUDE_KP * (TARGET_ALTITUDE - p1[2]),
                    -MAX_VERTICAL_SPEED,
                    MAX_VERTICAL_SPEED,
                ))
                vz2 = float(np.clip(
                    ALTITUDE_KP * (TARGET_ALTITUDE - p2[2]),
                    -MAX_VERTICAL_SPEED,
                    MAX_VERTICAL_SPEED,
                ))

                # Send commands.
                send_velocity(
                    cf1, cf1_xy[0], cf1_xy[1], vz1
                )
                send_velocity(
                    cf2, cf2_xy[0], cf2_xy[1], vz2
                )

                logger.writerow({
                    "time_s": round(elapsed, 3),
                    "cf1_x": p1[0],
                    "cf1_y": p1[1],
                    "cf1_z": p1[2],
                    "cf2_x": p2[0],
                    "cf2_y": p2[1],
                    "cf2_z": p2[2],
                    "goal_x": goal_xy[0],
                    "goal_y": goal_xy[1],
                    "center_x": center_xy[0],
                    "center_y": center_xy[1],
                    "goal_error_m": goal_error_norm,
                    "distance_m": distance,
                    "distance_error_m": distance_error,
                    "surprisal": surprisal,
                    "adaptive_kp": adaptive_kp,
                    "radial_velocity_m_s": radial_velocity,
                    "protective_mode": protective_mode,
                    "cf1_vx_cmd": cf1_xy[0],
                    "cf1_vy_cmd": cf1_xy[1],
                    "cf1_vz_cmd": vz1,
                    "cf2_vx_cmd": cf2_xy[0],
                    "cf2_vy_cmd": cf2_xy[1],
                    "cf2_vz_cmd": vz2,
                    "cf1_speed_est_m_s": float(
                        np.linalg.norm(v1)
                    ),
                    "cf2_speed_est_m_s": float(
                        np.linalg.norm(v2)
                    ),
                })
                file.flush()

                if elapsed >= next_print:
                    print(
                        f"t={elapsed:5.1f}s | "
                        f"goal_error={goal_error_norm:.3f}m | "
                        f"d={distance:.3f}m | "
                        f"d_error={distance_error:+.3f}m | "
                        f"S={surprisal:.3f} | "
                        f"Kp={adaptive_kp:.3f} | "
                        f"protect={protective_mode}"
                    )
                    next_print += 0.5

                if goal_error_norm <= GOAL_TOLERANCE:
                    print(
                        "\nFormation center reached the goal tolerance."
                    )
                    break

    finally:
        stop_motion(drones)

    print("\nNavigation experiment finished.")
    print(f"CSV saved to: {OUTPUT_CSV.resolve()}")
    print("Confirm that the fly_safely wrapper lands both drones.")


# ============================================================
# ENTRY POINT
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Real Crazyflie goal navigation with surprisal control."
    )
    parser.add_argument(
        "--mode",
        choices=["real"],
        default="real",
    )
    parser.parse_args()

    config = load_config()

    # Reuses the existing project's real-radio connection and
    # landing/cleanup workflow.
    fly_safely(config, 2, mission)


if __name__ == "__main__":
    main()
