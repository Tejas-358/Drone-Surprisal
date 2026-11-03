# """
# Lagrangian spring-damper formation control for 3 real Crazyflies (Lighthouse positioning).

# Usage
# -----
#   python crazyflie_formation.py --test-takeoff     # takeoff/hover/land of drone 1 only (debug)
#   python crazyflie_formation.py --dry-run          # connect, check status, no motor commands
#   python crazyflie_formation.py                    # full formation flight

# Approach
# --------
# Same equations as the simulation, evaluated on the MEASURED drone positions every step:
#   1. Forces (inter-drone springs/dampers + target attraction on the formation centre).
#   2. Acceleration = F / m  ->  integrated into a velocity command v_cmd (semi-implicit Euler).
#   3. v_cmd is sent to each drone as a world-frame velocity setpoint (speed-limited).

# Safety features
# ---------------
#   - Geofence (including a floor once airborne)
#   - 3D and horizontal minimum inter-drone distance (downwash)
#   - Stale-telemetry watchdog (aborts if any drone's state stops updating)
#   - Battery / supervisor / estimator preflight checks

# Requirements: pip install --upgrade cflib numpy matplotlib
# Close cfclient before running (it would hold the radio).
# Start every flight with all batteries above ~3.9 V.
# """

# import sys
# import time
# import threading
# from collections import deque
# from contextlib import ExitStack

# import numpy as np

# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.log import LogConfig

# # ==========================================================
# # 1. CONFIGURATION  (edit these)
# # ==========================================================

# URIS = [
#     "radio://0/80/2M/E7E7E7E7E7",
#     "radio://0/80/2M/E7E7E7E7E6",
#     "radio://0/80/2M/E7E7E7E7E5",
# ]
# # If you have several Crazyradios, give each drone its own radio index, e.g.
# # "radio://1/80/2M/...", "radio://2/80/2M/..."  (much better link quality)
# N = len(URIS)

# # Virtual dynamics (same structure as the simulation)
# m = 1.0
# k = 1.0                  # spring stiffness (softer than sim: real drones have lag)
# c = 2.5                  # damping (more than sim for the same reason)
# kt = 0.8
# ct = 1.5
# d_desired = 0.6          # keep >= 0.5 m for real Crazyflies (downwash)

# # If True, the damping terms use the drones' MEASURED velocity (stateEstimate.vx/vy)
# # instead of the integrated command v_cmd. Try this if you see oscillation / lag.
# USE_MEASURED_VELOCITY = False

# # Target relative to the formation centre at start (Lighthouse frame, metres)
# TARGET_OFFSET = np.array([0.5, 0.5])

# # Heights (staggered by 0.3 m so a drone is never directly in another's downwash)
# HEIGHTS = np.array([0.40, 0.40, 0.40])
# TAKEOFF_TIME = 3.0

# # Control loop
# CONTROL_HZ = 20.0
# DT = 1.0 / CONTROL_HZ
# FLIGHT_TIME = 15.0

# # Limits
# V_MAX = 0.4              # m/s per drone
# KZ = 1.0                 # altitude P gain
# VZ_MAX = 0.3

# # Safety. Geofence in Lighthouse frame: (xmin, xmax, ymin, ymax, zmax)
# GEOFENCE = (-2.0, 2.0, -2.0, 2.0, 1.5)
# MIN_FLIGHT_Z = 0.15              # abort if a drone drops below this while flying
# MIN_DRONE_DISTANCE = 0.25        # 3D distance
# MIN_HORIZONTAL_DISTANCE = 0.35   # horizontal distance (downwash protection)
# STALE_TIMEOUT = 0.3              # s; abort if any drone's state is older than this
# MIN_BATTERY_V = 1.0
# TAKEOFF_MIN_FRACTION = 0.6       # abort if below this fraction of target height after takeoff

# LOG_PERIOD_MS = 50


# # ==========================================================
# # 2. HELPERS: logging, arming, status
# # ==========================================================

# class StateStore:
#     """Thread-safe latest state (x, y, z, vx, vy) for each drone, with timestamps."""

#     def __init__(self, n):
#         self.lock = threading.Lock()
#         self.state = np.zeros((n, 5))
#         self.stamp = np.zeros(n)
#         self.updated = [False] * n

#     def callback(self, idx):
#         def _cb(timestamp, data, logconf):
#             with self.lock:
#                 self.state[idx] = [
#                     data["stateEstimate.x"],
#                     data["stateEstimate.y"],
#                     data["stateEstimate.z"],
#                     data["stateEstimate.vx"],
#                     data["stateEstimate.vy"],
#                 ]
#                 self.stamp[idx] = time.time()
#                 self.updated[idx] = True
#         return _cb

#     def get(self):
#         with self.lock:
#             return self.state.copy()

#     def all_ready(self):
#         with self.lock:
#             return all(self.updated)

#     def max_age(self):
#         """Age (s) of the OLDEST drone state."""
#         with self.lock:
#             return time.time() - self.stamp.min()


# def start_state_logging(cf, idx, store):
#     lg = LogConfig(name=f"state{idx}", period_in_ms=LOG_PERIOD_MS)
#     for var in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
#                 "stateEstimate.vx", "stateEstimate.vy"):
#         lg.add_variable(var, "float")
#     cf.log.add_config(lg)
#     lg.data_received_cb.add_callback(store.callback(idx))
#     lg.start()
#     return lg


# def _stop_log(lg):
#     try:
#         lg.stop()
#     except Exception:
#         pass
#     try:
#         lg.delete()
#     except Exception:
#         pass


# def wait_for_estimator(scf, timeout=20.0):
#     """Reset Kalman filter and wait until position variance has settled.

#     Callback based, so the timeout is enforced even if no log data arrives.
#     """
#     cf = scf.cf
#     cf.param.set_value("kalman.resetEstimation", "1")
#     time.sleep(0.1)
#     cf.param.set_value("kalman.resetEstimation", "0")

#     lg = LogConfig(name="KalmanVar", period_in_ms=100)
#     for var in ("kalman.varPX", "kalman.varPY", "kalman.varPZ"):
#         lg.add_variable(var, "float")

#     lock = threading.Lock()
#     windows = {key: deque(maxlen=10) for key in ("kalman.varPX", "kalman.varPY", "kalman.varPZ")}
#     threshold = 0.001

#     def cb(ts, data, conf):
#         with lock:
#             for key in windows:
#                 windows[key].append(data[key])

#     cf.log.add_config(lg)
#     lg.data_received_cb.add_callback(cb)
#     lg.start()
#     try:
#         t0 = time.time()
#         while time.time() - t0 < timeout:
#             with lock:
#                 if all(len(w) == 10 and (max(w) - min(w)) < threshold for w in windows.values()):
#                     return True
#             time.sleep(0.1)
#         return False
#     finally:
#         _stop_log(lg)


# def read_vars(scf, names, types, timeout=3.0):
#     """Read a few log variables once (with a real timeout)."""
#     lg = LogConfig(name="oneshot", period_in_ms=100)
#     for n, t in zip(names, types):
#         lg.add_variable(n, t)

#     got = {}
#     done = threading.Event()

#     def cb(ts, data, conf):
#         if not done.is_set():
#             got.update(data)
#             done.set()

#     scf.cf.log.add_config(lg)
#     lg.data_received_cb.add_callback(cb)
#     lg.start()
#     try:
#         if not done.wait(timeout):
#             raise TimeoutError("no log data received")
#         return dict(got)
#     finally:
#         _stop_log(lg)


# SUPERVISOR_BITS = [
#     "canBeArmed", "isArmed", "autoArm", "canFly", "isFlying", "isTumbled",
#     "isLocked", "isCrashed", "hlFlying", "hlTrajFinished", "hlDisabled",
# ]


# def decode_supervisor(info):
#     return {name: bool(info & (1 << i)) for i, name in enumerate(SUPERVISOR_BITS)}


# def print_status(scf, label):
#     """Print battery + supervisor state."""
#     try:
#         d = read_vars(scf, ["pm.vbat", "supervisor.info"], ["float", "uint16_t"])
#         sup = decode_supervisor(int(d["supervisor.info"]))
#         active = [n for n, v in sup.items() if v]
#         print(f"  {label}: battery {d['pm.vbat']:.2f} V | supervisor: {', '.join(active) or 'none'}")
#         return d["pm.vbat"], sup
#     except Exception as e:
#         print(f"  {label}: could not read supervisor info ({e})")
#         return None, {}


# def arm_drone(cf, label):
#     """Arm the drone (required by recent firmware, otherwise motors never spin)."""
#     if hasattr(cf, "supervisor") and hasattr(cf.supervisor, "send_arming_request"):
#         cf.supervisor.send_arming_request(True)
#         print(f"  {label}: arming request sent")
#     elif hasattr(cf, "platform") and hasattr(cf.platform, "send_arming_request"):
#         cf.platform.send_arming_request(True)
#         print(f"  {label}: arming request sent (legacy API)")
#     else:
#         print(f"  {label}: this cflib has no arming API (run: pip install --upgrade cflib)")
#     time.sleep(0.5)
#     # Unlock thrust protection on older firmware, then hand control back to high-level commander
#     cf.commander.send_setpoint(0, 0, 0, 0)
#     time.sleep(0.1)
#     cf.commander.send_notify_setpoint_stop()


# def prepare_drone(scf):
#     cf = scf.cf
#     cf.param.set_value("stabilizer.estimator", "2")      # Kalman
#     cf.param.set_value("commander.enHighLevel", "1")
#     cf.param.set_value("stabilizer.controller", "1")     # PID
#     time.sleep(0.1)


# # ==========================================================
# # 3. FORMATION DYNAMICS (same equations as the simulation)
# # ==========================================================

# def compute_forces(q, v, target):
#     """q, v: (N,2). Returns (N,2) forces."""
#     F = np.zeros((N, 2))
#     center = q.mean(axis=0)
#     center_vel = v.mean(axis=0)

#     for i in range(N):
#         for j in range(i + 1, N):
#             rel_pos = q[i] - q[j]
#             dist = np.linalg.norm(rel_pos)
#             if dist < 1e-6:
#                 continue
#             direction = rel_pos / dist
#             F_spring = -k * (dist - d_desired) * direction
#             F_damp = -c * (v[i] - v[j])
#             F_pair = F_spring + F_damp
#             F[i] += F_pair
#             F[j] -= F_pair

#     for i in range(N):
#         F[i] += -(kt / N) * (center - target) - (ct / N) * center_vel
#     return F


# def clip_norm(vec, max_norm):
#     n = np.linalg.norm(vec)
#     return vec if n <= max_norm else vec * (max_norm / n)


# def safety_violation(pos3, check_floor=True):
#     """Return a message if a safety limit is violated, else None.

#     check_floor=False is used for the on-ground start-position check.
#     """
#     xmin, xmax, ymin, ymax, zmax = GEOFENCE
#     for i, p in enumerate(pos3):
#         if not (xmin <= p[0] <= xmax and ymin <= p[1] <= ymax and p[2] <= zmax):
#             return f"Drone {i+1} left geofence: {np.round(p, 2)}"
#         if check_floor and p[2] < MIN_FLIGHT_Z:
#             return f"Drone {i+1} too low (z={p[2]:.2f} m)"
#     for i in range(N):
#         for j in range(i + 1, N):
#             diff = pos3[i] - pos3[j]
#             if np.linalg.norm(diff) < MIN_DRONE_DISTANCE:
#                 return f"Drones {i+1} and {j+1} too close (3D)"
#             if np.linalg.norm(diff[:2]) < MIN_HORIZONTAL_DISTANCE:
#                 return f"Drones {i+1} and {j+1} too close horizontally (downwash)"
#     return None


# # ==========================================================
# # 4. FLIGHT
# # ==========================================================

# def connect_all(stack, uris):
#     scfs = []
#     for uri in uris:
#         print(f"Connecting to {uri} ...")
#         scf = stack.enter_context(SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")))
#         scfs.append(scf)
#     return scfs


# def preflight(scfs):
#     ok = True
#     for scf in scfs:
#         prepare_drone(scf)
#     for i, scf in enumerate(scfs):
#         vbat, sup = print_status(scf, f"Drone {i+1}")
#         if vbat is None:
#             print(f"  !! Drone {i+1}: could not read battery - not flying")
#             ok = False
#         elif vbat < MIN_BATTERY_V:
#             print(f"  !! Drone {i+1} battery too low ({vbat:.2f} V) - swap it")
#             ok = False
#         if sup.get("isTumbled") or sup.get("isCrashed") or sup.get("isLocked"):
#             print(f"  !! Drone {i+1} is tumbled/crashed/locked - place it flat and power-cycle it")
#             ok = False
#     for i, scf in enumerate(scfs):
#         print(f"Waiting for position estimate of drone {i+1} ...")
#         if not wait_for_estimator(scf):
#             print(f"  !! Estimator of drone {i+1} did not converge (check Lighthouse visibility)")
#             ok = False
#     return ok


# def land_all(cfs, store=None, timeout=6.0):
#     """Land all drones. If a StateStore is given, wait until they are near the ground."""
#     print("Landing ...")
#     for cf in cfs:
#         try:
#             cf.commander.send_notify_setpoint_stop()
#             cf.high_level_commander.land(0.0, 3.0)
#         except Exception as e:
#             print("  land error:", e)

#     if store is not None:
#         t0 = time.time()
#         time.sleep(1.0)
#         while time.time() - t0 < timeout:
#             if store.get()[:, 2].max() < 0.08:
#                 break
#             time.sleep(0.1)
#         time.sleep(0.5)
#     else:
#         time.sleep(3.5)

#     for cf in cfs:
#         try:
#             cf.high_level_commander.stop()
#         except Exception:
#             pass


# def run_test_takeoff():
#     """Single drone (first URI): arm, takeoff, hover, land."""
#     cflib.crtp.init_drivers()
#     with SyncCrazyflie(URIS[0], cf=Crazyflie(rw_cache="./cache")) as scf:
#         cf = scf.cf
#         prepare_drone(scf)
#         print_status(scf, "Drone 1 (before)")
#         if not wait_for_estimator(scf):
#             print("Estimator did not converge - aborting")
#             return
#         arm_drone(cf, "Drone 1")
#         print_status(scf, "Drone 1 (after arming)")
#         input("Clear area. Press Enter to TAKE OFF ...")
#         try:
#             cf.high_level_commander.takeoff(0.5, 2.0)
#             time.sleep(4.0)
#             print_status(scf, "Drone 1 (hovering)")
#         finally:
#             land_all([cf])


# def run_formation(dry_run=False):
#     cflib.crtp.init_drivers()
#     store = StateStore(N)
#     log_t, log_q, log_z, log_v = [], [], [], []
#     target = None

#     with ExitStack() as stack:
#         scfs = connect_all(stack, URIS)
#         cfs = [s.cf for s in scfs]

#         if not preflight(scfs):
#             print("\nPre-flight checks FAILED - not flying.")
#             return
#         if dry_run:
#             print("\nDry run OK - connections, battery, supervisor and estimators look fine.")
#             return

#         log_configs = [start_state_logging(cf, i, store) for i, cf in enumerate(cfs)]
#         t0 = time.time()
#         while not store.all_ready():
#             if time.time() - t0 > 5:
#                 print("No state data received from all drones - not flying.")
#                 return
#             time.sleep(0.05)

#         start_state = store.get()
#         start_center = start_state[:, :2].mean(axis=0)
#         target = start_center + TARGET_OFFSET
#         print(f"Start positions:\n{np.round(start_state[:, :3], 2)}")
#         print(f"Start centre: {np.round(start_center, 2)}, target: {np.round(target, 2)}")

#         x_lo, x_hi, y_lo, y_hi, _ = GEOFENCE
#         if not (x_lo < target[0] < x_hi and y_lo < target[1] < y_hi):
#             print("Target is outside the geofence - adjust TARGET_OFFSET or GEOFENCE")
#             return
#         msg = safety_violation(start_state[:, :3], check_floor=False)
#         if msg:
#             print("Start-position check failed:", msg)
#             return

#         flying = False
#         try:
#             for i, cf in enumerate(cfs):
#                 arm_drone(cf, f"Drone {i+1}")
#             for i, scf in enumerate(scfs):
#                 print_status(scf, f"Drone {i+1} (armed)")

#             input("\nArea clear? Press Enter to TAKE OFF (Ctrl+C aborts) ...")

#             for i, cf in enumerate(cfs):
#                 cf.high_level_commander.takeoff(float(HEIGHTS[i]), TAKEOFF_TIME)
#             flying = True
#             time.sleep(TAKEOFF_TIME + 1.5)

#             if store.max_age() > STALE_TIMEOUT:
#                 raise RuntimeError("Stale state data after takeoff - aborting")

#             meas = store.get()
#             for i in range(N):
#                 if meas[i, 2] < TAKEOFF_MIN_FRACTION * HEIGHTS[i]:
#                     print(f"Drone {i+1} did not take off (z={meas[i,2]:.2f} m). Supervisor state:")
#                     for j, scf in enumerate(scfs):
#                         print_status(scf, f"Drone {j+1}")
#                     raise RuntimeError("Takeoff failed - aborting")

#             # Velocity command state (equivalent to the simulation's velocity v)
#             v_cmd = np.zeros((N, 2))

#             print("Formation control running ...")
#             t_start = time.time()
#             t_prev = t_start
#             next_tick = t_start
#             last_print = -10.0

#             while True:
#                 now = time.time()
#                 t = now - t_start
#                 if t > FLIGHT_TIME:
#                     break
#                 dt = max(1e-3, min(now - t_prev, 0.1))
#                 t_prev = now

#                 # Watchdog: telemetry must be fresh for ALL drones
#                 age = store.max_age()
#                 if age > STALE_TIMEOUT:
#                     print(f"SAFETY ABORT: stale state data ({age:.2f} s old)")
#                     break

#                 meas = store.get()
#                 q_meas = meas[:, :2]
#                 v_meas = meas[:, 3:5]
#                 z_meas = meas[:, 2]

#                 msg = safety_violation(meas[:, :3])
#                 if msg:
#                     print("SAFETY ABORT:", msg)
#                     break

#                 # Same equations as the simulation, using MEASURED positions
#                 v_for_forces = v_meas if USE_MEASURED_VELOCITY else v_cmd
#                 F = compute_forces(q_meas, v_for_forces, target)
#                 v_cmd = v_cmd + (F / m) * dt
#                 v_cmd = np.array([clip_norm(v, V_MAX) for v in v_cmd])

#                 for i, cf in enumerate(cfs):
#                     vz = float(np.clip(KZ * (HEIGHTS[i] - z_meas[i]), -VZ_MAX, VZ_MAX))
#                     cf.commander.send_velocity_world_setpoint(
#                         float(v_cmd[i, 0]), float(v_cmd[i, 1]), vz, 0.0
#                     )

#                 log_t.append(t)
#                 log_q.append(q_meas.copy())
#                 log_z.append(z_meas.copy())
#                 log_v.append(v_cmd.copy())

#                 if t - last_print > 2.0:
#                     last_print = t
#                     d = [np.linalg.norm(q_meas[i] - q_meas[j]) for i, j in [(0, 1), (1, 2), (2, 0)]]
#                     print(f"  t={t:5.1f}s  dists={np.round(d, 2)}  centre={np.round(q_meas.mean(axis=0), 2)}")

#                 next_tick += DT
#                 sleep = next_tick - time.time()
#                 if sleep > 0:
#                     time.sleep(sleep)

#         except KeyboardInterrupt:
#             print("\nInterrupted - landing.")
#         except Exception as e:
#             print("ERROR:", e, "- landing.")
#         finally:
#             if flying:
#                 land_all(cfs, store)
#             for lg in log_configs:
#                 _stop_log(lg)

#     if log_t:
#         log_t = np.array(log_t)
#         log_q = np.array(log_q)
#         log_z = np.array(log_z)
#         np.savez("formation_flight_log.npz", t=log_t, q=log_q, z=log_z,
#                  v_cmd=np.array(log_v), target=target)
#         print("Saved formation_flight_log.npz")
#         plot_results(log_t, log_q, log_z, target)


# def plot_results(t, q, z, target):
#     import matplotlib.pyplot as plt

#     colors = ["red", "blue", "green"]
#     fig, ax = plt.subplots(1, 3, figsize=(20, 6))

#     for i in range(N):
#         ax[0].plot(q[:, i, 0], q[:, i, 1], color=colors[i], label=f"Drone {i+1}")
#         ax[0].plot(q[-1, i, 0], q[-1, i, 1], "o", color=colors[i])
#     ax[0].plot(*target, "k*", markersize=16, label="Target")
#     ax[0].set_aspect("equal")
#     ax[0].grid(True)
#     ax[0].set_xlabel("X (m)")
#     ax[0].set_ylabel("Y (m)")
#     ax[0].set_title("Trajectories (Lighthouse frame)")
#     ax[0].legend()

#     for i, j in [(0, 1), (1, 2), (2, 0)]:
#         ax[1].plot(t, np.linalg.norm(q[:, i] - q[:, j], axis=1), label=f"D{i+1}-D{j+1}")
#     ax[1].axhline(d_desired, color="k", linestyle="--", label="Desired distance")
#     ax[1].set_xlabel("Time (s)")
#     ax[1].set_ylabel("Distance (m)")
#     ax[1].set_title("Inter-drone distance convergence")
#     ax[1].grid(True)
#     ax[1].legend()

#     for i in range(N):
#         ax[2].plot(t, z[:, i], color=colors[i], label=f"Drone {i+1}")
#         ax[2].axhline(HEIGHTS[i], color=colors[i], linestyle=":", alpha=0.6)
#     ax[2].set_xlabel("Time (s)")
#     ax[2].set_ylabel("Height (m)")
#     ax[2].set_title("Altitude (dotted = setpoint)")
#     ax[2].grid(True)
#     ax[2].legend()

#     plt.tight_layout()
#     plt.show()


# if __name__ == "__main__":
#     if "--test-takeoff" in sys.argv:
#         run_test_takeoff()
#     elif "--dry-run" in sys.argv:
#         run_formation(dry_run=True)
#     else:
#         run_formation()


"""
Lagrangian spring-damper formation control for 3 real Crazyflies (Lighthouse positioning).

Usage
-----
  python crazyflie_formation.py --test-takeoff     # takeoff/hover/land of drone 1 only (debug)
  python crazyflie_formation.py --dry-run          # connect, check status, no motor commands
  python crazyflie_formation.py                    # full formation flight (asks for the target)
  python crazyflie_formation.py --target 1.0 -0.5  # fly the formation centre to x=1.0, y=-0.5

Approach
--------
Same equations as the simulation, evaluated on the MEASURED drone positions every step:
  1. Forces (inter-drone springs/dampers + target attraction on the formation centre).
  2. Acceleration = F / m  ->  integrated into a velocity command v_cmd (semi-implicit Euler).
  3. v_cmd is sent to each drone as a world-frame velocity setpoint (speed-limited).

Safety features
---------------
  - Geofence (including a floor once airborne)
  - 3D and horizontal minimum inter-drone distance (downwash)
  - Stale-telemetry watchdog (aborts if any drone's state stops updating)
  - Battery / supervisor / estimator preflight checks

Requirements: pip install --upgrade cflib numpy matplotlib
Close cfclient before running (it would hold the radio).
Start every flight with all batteries above ~3.9 V.
"""

import sys
import time
import threading
from collections import deque
from contextlib import ExitStack

import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# ==========================================================
# 1. CONFIGURATION  (edit these)
# ==========================================================

URIS = [
    "radio://0/80/2M/E7E7E7E7E7",
    "radio://0/80/2M/E7E7E7E7E6",
    "radio://0/80/2M/E7E7E7E7E5",
]
# If you have several Crazyradios, give each drone its own radio index, e.g.
# "radio://1/80/2M/...", "radio://2/80/2M/..."  (much better link quality)
N = len(URIS)

# Virtual dynamics (same structure as the simulation)
m = 1.0
k = 1.0                  # spring stiffness (softer than sim: real drones have lag)
c = 2.5                  # damping (more than sim for the same reason)
kt = 0.8
ct = 1.5
d_desired = 0.6          # keep >= 0.5 m for real Crazyflies (downwash)

# If True, the damping terms use the drones' MEASURED velocity (stateEstimate.vx/vy)
# instead of the integrated command v_cmd. Try this if you see oscillation / lag.
USE_MEASURED_VELOCITY = False

# Target relative to the formation centre at start (Lighthouse frame, metres)
TARGET_OFFSET = np.array([0.5, 0.5])   # default if you just press Enter at the prompt

# Exact target for the formation CENTRE in Lighthouse coordinates (metres), e.g. (1.0, -0.5).
# Priority: --target X Y on the command line > TARGET_XY below > interactive prompt.
TARGET_XY = None

# Keep the target this far inside the geofence so the whole triangle (circumradius
# ~0.35 m) stays inside it when the centre arrives.
TARGET_MARGIN = 0.4

# Heights (staggered by 0.3 m so a drone is never directly in another's downwash)
HEIGHTS = np.array([0.50, 0.50, 0.50])
TAKEOFF_TIME = 3.0

# Control loop
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
FLIGHT_TIME = 15.0

# Limits
V_MAX = 0.4              # m/s per drone
KZ = 1.0                 # altitude P gain
VZ_MAX = 0.3

# Safety. Geofence in Lighthouse frame: (xmin, xmax, ymin, ymax, zmax)
GEOFENCE = (-2.0, 2.0, -2.0, 2.0, 1.5)
MIN_FLIGHT_Z = 0.15              # abort if a drone drops below this while flying
MIN_DRONE_DISTANCE = 0.25        # 3D distance
MIN_HORIZONTAL_DISTANCE = 0.35   # horizontal distance (downwash protection)
STALE_TIMEOUT = 0.3              # s; abort if any drone's state is older than this
MIN_BATTERY_V = 1.0
TAKEOFF_MIN_FRACTION = 0.6       # abort if below this fraction of target height after takeoff

LOG_PERIOD_MS = 50


# ==========================================================
# 2. HELPERS: logging, arming, status
# ==========================================================

class StateStore:
    """Thread-safe latest state (x, y, z, vx, vy) for each drone, with timestamps."""

    def __init__(self, n):
        self.lock = threading.Lock()
        self.state = np.zeros((n, 5))
        self.stamp = np.zeros(n)
        self.updated = [False] * n

    def callback(self, idx):
        def _cb(timestamp, data, logconf):
            with self.lock:
                self.state[idx] = [
                    data["stateEstimate.x"],
                    data["stateEstimate.y"],
                    data["stateEstimate.z"],
                    data["stateEstimate.vx"],
                    data["stateEstimate.vy"],
                ]
                self.stamp[idx] = time.time()
                self.updated[idx] = True
        return _cb

    def get(self):
        with self.lock:
            return self.state.copy()

    def all_ready(self):
        with self.lock:
            return all(self.updated)

    def max_age(self):
        """Age (s) of the OLDEST drone state."""
        with self.lock:
            return time.time() - self.stamp.min()


def start_state_logging(cf, idx, store):
    lg = LogConfig(name=f"state{idx}", period_in_ms=LOG_PERIOD_MS)
    for var in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                "stateEstimate.vx", "stateEstimate.vy"):
        lg.add_variable(var, "float")
    cf.log.add_config(lg)
    lg.data_received_cb.add_callback(store.callback(idx))
    lg.start()
    return lg


def _stop_log(lg):
    try:
        lg.stop()
    except Exception:
        pass
    try:
        lg.delete()
    except Exception:
        pass


def wait_for_estimator(scf, timeout=20.0):
    """Reset Kalman filter and wait until position variance has settled.

    Callback based, so the timeout is enforced even if no log data arrives.
    """
    cf = scf.cf
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")

    lg = LogConfig(name="KalmanVar", period_in_ms=100)
    for var in ("kalman.varPX", "kalman.varPY", "kalman.varPZ"):
        lg.add_variable(var, "float")

    lock = threading.Lock()
    windows = {key: deque(maxlen=10) for key in ("kalman.varPX", "kalman.varPY", "kalman.varPZ")}
    threshold = 0.001

    def cb(ts, data, conf):
        with lock:
            for key in windows:
                windows[key].append(data[key])

    cf.log.add_config(lg)
    lg.data_received_cb.add_callback(cb)
    lg.start()
    try:
        t0 = time.time()
        while time.time() - t0 < timeout:
            with lock:
                if all(len(w) == 10 and (max(w) - min(w)) < threshold for w in windows.values()):
                    return True
            time.sleep(0.1)
        return False
    finally:
        _stop_log(lg)


def read_vars(scf, names, types, timeout=3.0):
    """Read a few log variables once (with a real timeout)."""
    lg = LogConfig(name="oneshot", period_in_ms=100)
    for n, t in zip(names, types):
        lg.add_variable(n, t)

    got = {}
    done = threading.Event()

    def cb(ts, data, conf):
        if not done.is_set():
            got.update(data)
            done.set()

    scf.cf.log.add_config(lg)
    lg.data_received_cb.add_callback(cb)
    lg.start()
    try:
        if not done.wait(timeout):
            raise TimeoutError("no log data received")
        return dict(got)
    finally:
        _stop_log(lg)


SUPERVISOR_BITS = [
    "canBeArmed", "isArmed", "autoArm", "canFly", "isFlying", "isTumbled",
    "isLocked", "isCrashed", "hlFlying", "hlTrajFinished", "hlDisabled",
]


def decode_supervisor(info):
    return {name: bool(info & (1 << i)) for i, name in enumerate(SUPERVISOR_BITS)}


def print_status(scf, label):
    """Print battery + supervisor state."""
    try:
        d = read_vars(scf, ["pm.vbat", "supervisor.info"], ["float", "uint16_t"])
        sup = decode_supervisor(int(d["supervisor.info"]))
        active = [n for n, v in sup.items() if v]
        print(f"  {label}: battery {d['pm.vbat']:.2f} V | supervisor: {', '.join(active) or 'none'}")
        return d["pm.vbat"], sup
    except Exception as e:
        print(f"  {label}: could not read supervisor info ({e})")
        return None, {}


def arm_drone(cf, label):
    """Arm the drone (required by recent firmware, otherwise motors never spin)."""
    if hasattr(cf, "supervisor") and hasattr(cf.supervisor, "send_arming_request"):
        cf.supervisor.send_arming_request(True)
        print(f"  {label}: arming request sent")
    elif hasattr(cf, "platform") and hasattr(cf.platform, "send_arming_request"):
        cf.platform.send_arming_request(True)
        print(f"  {label}: arming request sent (legacy API)")
    else:
        print(f"  {label}: this cflib has no arming API (run: pip install --upgrade cflib)")
    time.sleep(0.5)
    # Unlock thrust protection on older firmware, then hand control back to high-level commander
    cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)
    cf.commander.send_notify_setpoint_stop()


def prepare_drone(scf):
    cf = scf.cf
    cf.param.set_value("stabilizer.estimator", "2")      # Kalman
    cf.param.set_value("commander.enHighLevel", "1")
    cf.param.set_value("stabilizer.controller", "1")     # PID
    time.sleep(0.1)


# ==========================================================
# 3. FORMATION DYNAMICS (same equations as the simulation)
# ==========================================================

def compute_forces(q, v, target):
    """q, v: (N,2). Returns (N,2) forces."""
    F = np.zeros((N, 2))
    center = q.mean(axis=0)
    center_vel = v.mean(axis=0)

    for i in range(N):
        for j in range(i + 1, N):
            rel_pos = q[i] - q[j]
            dist = np.linalg.norm(rel_pos)
            if dist < 1e-6:
                continue
            direction = rel_pos / dist
            F_spring = -k * (dist - d_desired) * direction
            F_damp = -c * (v[i] - v[j])
            F_pair = F_spring + F_damp
            F[i] += F_pair
            F[j] -= F_pair

    for i in range(N):
        F[i] += -(kt / N) * (center - target) - (ct / N) * center_vel
    return F


def clip_norm(vec, max_norm):
    n = np.linalg.norm(vec)
    return vec if n <= max_norm else vec * (max_norm / n)


def safety_violation(pos3, check_floor=True):
    """Return a message if a safety limit is violated, else None.

    check_floor=False is used for the on-ground start-position check.
    """
    xmin, xmax, ymin, ymax, zmax = GEOFENCE
    for i, p in enumerate(pos3):
        if not (xmin <= p[0] <= xmax and ymin <= p[1] <= ymax and p[2] <= zmax):
            return f"Drone {i+1} left geofence: {np.round(p, 2)}"
        if check_floor and p[2] < MIN_FLIGHT_Z:
            return f"Drone {i+1} too low (z={p[2]:.2f} m)"
    for i in range(N):
        for j in range(i + 1, N):
            diff = pos3[i] - pos3[j]
            if np.linalg.norm(diff) < MIN_DRONE_DISTANCE:
                return f"Drones {i+1} and {j+1} too close (3D)"
            if np.linalg.norm(diff[:2]) < MIN_HORIZONTAL_DISTANCE:
                return f"Drones {i+1} and {j+1} too close horizontally (downwash)"
    return None


# ==========================================================
# 4. FLIGHT
# ==========================================================

def parse_target_arg():
    """Return np.array([x, y]) from '--target X Y' on the command line, or None if absent."""
    if "--target" not in sys.argv:
        return None
    i = sys.argv.index("--target")
    try:
        return np.array([float(sys.argv[i + 1]), float(sys.argv[i + 2])])
    except (IndexError, ValueError):
        print("Usage: --target X Y   (metres, Lighthouse frame, e.g. --target 1.0 -0.5)")
        sys.exit(1)


def target_in_bounds(target):
    x_lo, x_hi, y_lo, y_hi, _ = GEOFENCE
    mg = TARGET_MARGIN
    return (x_lo + mg <= target[0] <= x_hi - mg) and (y_lo + mg <= target[1] <= y_hi - mg)


def choose_target(start_center):
    """Get the exact target for the formation centre.

    Returns the target (np.array of 2) or None if a pre-set target is invalid.
    """
    preset = parse_target_arg()
    if preset is None and TARGET_XY is not None:
        preset = np.array(TARGET_XY, dtype=float)

    if preset is not None:
        if not target_in_bounds(preset):
            print(f"Target {np.round(preset, 2)} is outside the allowed area "
                  f"(geofence minus {TARGET_MARGIN} m margin).")
            return None
        return preset

    default = start_center + TARGET_OFFSET
    x_lo, x_hi, y_lo, y_hi, _ = GEOFENCE
    print(f"\nAllowed target area: x in [{x_lo + TARGET_MARGIN:.1f}, {x_hi - TARGET_MARGIN:.1f}], "
          f"y in [{y_lo + TARGET_MARGIN:.1f}, {y_hi - TARGET_MARGIN:.1f}] (Lighthouse frame, metres)")
    while True:
        raw = input(f"Enter target 'x y' for the formation centre "
                    f"(Enter = default {np.round(default, 2)}): ").strip()
        if raw == "":
            target = default
        else:
            try:
                x, y = [float(s) for s in raw.replace(",", " ").split()]
                target = np.array([x, y])
            except ValueError:
                print("  Please enter two numbers, e.g.  1.0 -0.5")
                continue
        if not target_in_bounds(target):
            print("  Outside the allowed area - try again.")
            continue
        return target


def connect_all(stack, uris):
    scfs = []
    for uri in uris:
        print(f"Connecting to {uri} ...")
        scf = stack.enter_context(SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")))
        scfs.append(scf)
    return scfs


def preflight(scfs):
    ok = True
    for scf in scfs:
        prepare_drone(scf)
    for i, scf in enumerate(scfs):
        vbat, sup = print_status(scf, f"Drone {i+1}")
        if vbat is None:
            print(f"  !! Drone {i+1}: could not read battery - not flying")
            ok = False
        elif vbat < MIN_BATTERY_V:
            print(f"  !! Drone {i+1} battery too low ({vbat:.2f} V) - swap it")
            ok = False
        if sup.get("isTumbled") or sup.get("isCrashed") or sup.get("isLocked"):
            print(f"  !! Drone {i+1} is tumbled/crashed/locked - place it flat and power-cycle it")
            ok = False
    for i, scf in enumerate(scfs):
        print(f"Waiting for position estimate of drone {i+1} ...")
        if not wait_for_estimator(scf):
            print(f"  !! Estimator of drone {i+1} did not converge (check Lighthouse visibility)")
            ok = False
    return ok


def land_all(cfs, store=None, timeout=6.0):
    """Land all drones. If a StateStore is given, wait until they are near the ground."""
    print("Landing ...")
    for cf in cfs:
        try:
            cf.commander.send_notify_setpoint_stop()
            cf.high_level_commander.land(0.0, 3.0)
        except Exception as e:
            print("  land error:", e)

    if store is not None:
        t0 = time.time()
        time.sleep(1.0)
        while time.time() - t0 < timeout:
            if store.get()[:, 2].max() < 0.08:
                break
            time.sleep(0.1)
        time.sleep(0.5)
    else:
        time.sleep(3.5)

    for cf in cfs:
        try:
            cf.high_level_commander.stop()
        except Exception:
            pass


def run_test_takeoff():
    """Single drone (first URI): arm, takeoff, hover, land."""
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URIS[0], cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        prepare_drone(scf)
        print_status(scf, "Drone 1 (before)")
        if not wait_for_estimator(scf):
            print("Estimator did not converge - aborting")
            return
        arm_drone(cf, "Drone 1")
        print_status(scf, "Drone 1 (after arming)")
        input("Clear area. Press Enter to TAKE OFF ...")
        try:
            cf.high_level_commander.takeoff(0.5, 2.0)
            time.sleep(4.0)
            print_status(scf, "Drone 1 (hovering)")
        finally:
            land_all([cf])


def run_formation(dry_run=False):
    cflib.crtp.init_drivers()
    store = StateStore(N)
    log_t, log_q, log_z, log_v = [], [], [], []
    target = None

    with ExitStack() as stack:
        scfs = connect_all(stack, URIS)
        cfs = [s.cf for s in scfs]

        if not preflight(scfs):
            print("\nPre-flight checks FAILED - not flying.")
            return
        if dry_run:
            print("\nDry run OK - connections, battery, supervisor and estimators look fine.")
            return

        log_configs = [start_state_logging(cf, i, store) for i, cf in enumerate(cfs)]
        t0 = time.time()
        while not store.all_ready():
            if time.time() - t0 > 5:
                print("No state data received from all drones - not flying.")
                return
            time.sleep(0.05)

        start_state = store.get()
        start_center = start_state[:, :2].mean(axis=0)
        print(f"Start positions:\n{np.round(start_state[:, :3], 2)}")
        print(f"Start centre: {np.round(start_center, 2)}")

        msg = safety_violation(start_state[:, :3], check_floor=False)
        if msg:
            print("Start-position check failed:", msg)
            return

        target = choose_target(start_center)
        if target is None:
            return
        distance = np.linalg.norm(target - start_center)
        print(f"Target: {np.round(target, 2)}  (distance {distance:.2f} m)")
        # Rough lower bound on travel time at full speed; the spring-damper dynamics are slower.
        if distance / V_MAX * 1.5 > FLIGHT_TIME:
            print(f"  Warning: FLIGHT_TIME={FLIGHT_TIME:.0f} s may be too short to reach this target "
                  f"(speed limit {V_MAX} m/s). Consider increasing FLIGHT_TIME.")

        flying = False
        try:
            for i, cf in enumerate(cfs):
                arm_drone(cf, f"Drone {i+1}")
            for i, scf in enumerate(scfs):
                print_status(scf, f"Drone {i+1} (armed)")

            input("\nArea clear? Press Enter to TAKE OFF (Ctrl+C aborts) ...")

            for i, cf in enumerate(cfs):
                cf.high_level_commander.takeoff(float(HEIGHTS[i]), TAKEOFF_TIME)
            flying = True
            time.sleep(TAKEOFF_TIME + 1.5)

            if store.max_age() > STALE_TIMEOUT:
                raise RuntimeError("Stale state data after takeoff - aborting")

            meas = store.get()
            for i in range(N):
                if meas[i, 2] < TAKEOFF_MIN_FRACTION * HEIGHTS[i]:
                    print(f"Drone {i+1} did not take off (z={meas[i,2]:.2f} m). Supervisor state:")
                    for j, scf in enumerate(scfs):
                        print_status(scf, f"Drone {j+1}")
                    raise RuntimeError("Takeoff failed - aborting")

            # Velocity command state (equivalent to the simulation's velocity v)
            v_cmd = np.zeros((N, 2))

            print("Formation control running ...")
            t_start = time.time()
            t_prev = t_start
            next_tick = t_start
            last_print = -10.0

            while True:
                now = time.time()
                t = now - t_start
                if t > FLIGHT_TIME:
                    break
                dt = max(1e-3, min(now - t_prev, 0.1))
                t_prev = now

                # Watchdog: telemetry must be fresh for ALL drones
                age = store.max_age()
                if age > STALE_TIMEOUT:
                    print(f"SAFETY ABORT: stale state data ({age:.2f} s old)")
                    break

                meas = store.get()
                q_meas = meas[:, :2]
                v_meas = meas[:, 3:5]
                z_meas = meas[:, 2]

                msg = safety_violation(meas[:, :3])
                if msg:
                    print("SAFETY ABORT:", msg)
                    break

                # Same equations as the simulation, using MEASURED positions
                v_for_forces = v_meas if USE_MEASURED_VELOCITY else v_cmd
                F = compute_forces(q_meas, v_for_forces, target)
                v_cmd = v_cmd + (F / m) * dt
                v_cmd = np.array([clip_norm(v, V_MAX) for v in v_cmd])

                for i, cf in enumerate(cfs):
                    vz = float(np.clip(KZ * (HEIGHTS[i] - z_meas[i]), -VZ_MAX, VZ_MAX))
                    cf.commander.send_velocity_world_setpoint(
                        float(v_cmd[i, 0]), float(v_cmd[i, 1]), vz, 0.0
                    )

                log_t.append(t)
                log_q.append(q_meas.copy())
                log_z.append(z_meas.copy())
                log_v.append(v_cmd.copy())

                if t - last_print > 2.0:
                    last_print = t
                    d = [np.linalg.norm(q_meas[i] - q_meas[j]) for i, j in [(0, 1), (1, 2), (2, 0)]]
                    print(f"  t={t:5.1f}s  dists={np.round(d, 2)}  centre={np.round(q_meas.mean(axis=0), 2)}")

                next_tick += DT
                sleep = next_tick - time.time()
                if sleep > 0:
                    time.sleep(sleep)

        except KeyboardInterrupt:
            print("\nInterrupted - landing.")
        except Exception as e:
            print("ERROR:", e, "- landing.")
        finally:
            if flying:
                land_all(cfs, store)
            for lg in log_configs:
                _stop_log(lg)

    if log_t:
        log_t = np.array(log_t)
        log_q = np.array(log_q)
        log_z = np.array(log_z)
        np.savez("formation_flight_log.npz", t=log_t, q=log_q, z=log_z,
                 v_cmd=np.array(log_v), target=target)
        print("Saved formation_flight_log.npz")
        final_err = np.linalg.norm(log_q[-1].mean(axis=0) - target)
        print(f"Final centre error: {final_err:.2f} m")
        plot_results(log_t, log_q, log_z, target)


def plot_results(t, q, z, target):
    import matplotlib.pyplot as plt

    colors = ["red", "blue", "green"]
    fig, ax = plt.subplots(1, 3, figsize=(20, 6))

    for i in range(N):
        ax[0].plot(q[:, i, 0], q[:, i, 1], color=colors[i], label=f"Drone {i+1}")
        ax[0].plot(q[-1, i, 0], q[-1, i, 1], "o", color=colors[i])
    ax[0].plot(*target, "k*", markersize=16, label="Target")
    ax[0].set_aspect("equal")
    ax[0].grid(True)
    ax[0].set_xlabel("X (m)")
    ax[0].set_ylabel("Y (m)")
    ax[0].set_title("Trajectories (Lighthouse frame)")
    ax[0].legend()

    for i, j in [(0, 1), (1, 2), (2, 0)]:
        ax[1].plot(t, np.linalg.norm(q[:, i] - q[:, j], axis=1), label=f"D{i+1}-D{j+1}")
    ax[1].axhline(d_desired, color="k", linestyle="--", label="Desired distance")
    ax[1].set_xlabel("Time (s)")
    ax[1].set_ylabel("Distance (m)")
    ax[1].set_title("Inter-drone distance convergence")
    ax[1].grid(True)
    ax[1].legend()

    for i in range(N):
        ax[2].plot(t, z[:, i], color=colors[i], label=f"Drone {i+1}")
        ax[2].axhline(HEIGHTS[i], color=colors[i], linestyle=":", alpha=0.6)
    ax[2].set_xlabel("Time (s)")
    ax[2].set_ylabel("Height (m)")
    ax[2].set_title("Altitude (dotted = setpoint)")
    ax[2].grid(True)
    ax[2].legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if "--test-takeoff" in sys.argv:
        run_test_takeoff()
    elif "--dry-run" in sys.argv:
        run_formation(dry_run=True)
    else:
        run_formation()