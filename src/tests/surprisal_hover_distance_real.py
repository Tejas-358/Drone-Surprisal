
"""
surprisal_hover_distance_real.py

Two Crazyflies hover and maintain their initial separation.
The drone that appears less displaced becomes the responder.

Safety:
- Test in CrazySim first.
- Do not push a drone by hand while its propellers are spinning.
- This is an experimental controller, not a collision-prevention guarantee.
"""

import time
import csv
from pathlib import Path

import numpy as np

from src.utils.connection import load_config
from src.utils.flight import fly_safely


# ----------------------- Experiment settings -----------------------

NUM_DRONES = 2

HOVER_Z = 0.40
MIN_Z = 0.05
MAX_Z = 1.00

# Keep the separation measured at startup.
DESIRED_DISTANCE = None

# Stop the experiment if the drones become too close.
CRITICAL_DISTANCE = 0.20
WARNING_DISTANCE = 0.40

# Conservative horizontal velocity limit.
MAX_SPEED = 0.08

# Adaptive surprisal controller.
BASE_KP = 0.20
SURPRISE_GAIN = 1.0
MAX_KP = 0.40
SIGMA_DISTANCE = 0.10

# Relative-velocity damping.
KD_RELATIVE = 0.20

# Detect displacement from the initial hover positions.
DISPLACEMENT_THRESHOLD = 0.06

# Stop if either drone strays too far from its initial position.
MAX_DISPLACEMENT = 0.80

CONTROL_DT = 0.05
MAX_DURATION = 45.0

LOG_PATH = Path("data/raw/surprisal_hover_distance_real.csv")


# ----------------------- Small helpers ----------------------------

def limit_vector(vector, max_norm):
    """Limit a 2-D vector's magnitude."""
    norm = float(np.linalg.norm(vector))
    if norm > max_norm and norm > 1e-9:
        return vector * (max_norm / norm)
    return vector


def estimate_velocity(position, previous_position, dt):
    if previous_position is None or dt <= 0:
        return np.zeros(3)
    return (position - previous_position) / dt


def calculate_surprisal(distance, target_distance):
    """
    Gaussian-style distance surprisal.
    Larger deviations from target produce larger surprisal.
    """
    error = distance - target_distance
    return 0.5 * (error / SIGMA_DISTANCE) ** 2


def adaptive_gain(surprisal):
    return min(
        BASE_KP * (1.0 + SURPRISE_GAIN * surprisal),
        MAX_KP,
    )


def send_hover_velocity(scf, vx, vy):
    """Send a world-frame velocity setpoint to a Crazyflie."""
    scf.cf.commander.send_velocity_world_setpoint(
        float(vx),
        float(vy),
        0.0,  # vertical velocity
        0.0,  # yaw rate
    )


def read_position(state):
    return np.array(
        [state["x"], state["y"], state["z"]],
        dtype=float,
    )


# ----------------------- Control loop -----------------------------

def mission(drones, config, states):
    """
    Expected:
      drones: two connected Crazyflie objects
      states: latest Lighthouse position state for each drone

    Match the state lookup below to your project's working test.
    
    """

    print(f"Number of drone state entries: {len(states)}")
    print("CF1 state:", states[0])
    print("CF2 state:", states[1])

    if len(drones) != 2:
        raise RuntimeError("This experiment requires exactly two drones.")

    cf1, cf2 = drones

    # Adjust these keys if your states dictionary uses radio URIs or IDs.
    def get_positions():
        p1 = read_position(states[0])
        p2 = read_position(states[1])
        return p1, p2

    print("\nKeep the flight area clear. Do not touch either drone.")
    print("Waiting briefly for stable Lighthouse position readings...")
    time.sleep(2.0)

    p1, p2 = get_positions()

    if not (MIN_Z <= p1[2] <= MAX_Z and MIN_Z <= p2[2] <= MAX_Z):
        raise RuntimeError(
            "Initial altitude is outside the configured safety envelope."
        )

    initial_1 = p1.copy()
    initial_2 = p2.copy()

    target_distance = float(np.linalg.norm(p2[:2] - p1[:2]))
    if target_distance < WARNING_DISTANCE:
        raise RuntimeError(
            f"Initial separation {target_distance:.2f} m is too small. "
            "Reposition the drones before starting."
        )

    print(f"Initial target separation: {target_distance:.3f} m")
    print("Starting hover and distance-maintenance control.")

    previous_1 = None
    previous_2 = None
    previous_time = time.monotonic()
    start_time = previous_time

    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)

    with LOG_PATH.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow([
            "time_s",
            "distance_m",
            "distance_error_m",
            "surprisal",
            "adaptive_kp",
            "displacement_1_m",
            "displacement_2_m",
            "responder",
            "cmd1_vx",
            "cmd1_vy",
            "cmd2_vx",
            "cmd2_vy",
        ])

        try:
            while time.monotonic() - start_time < MAX_DURATION:
                now = time.monotonic()
                dt = max(now - previous_time, 1e-3)
                previous_time = now

                p1, p2 = get_positions()

                # Basic altitude and displacement safety checks.
                if not (
                    MIN_Z <= p1[2] <= MAX_Z
                    and MIN_Z <= p2[2] <= MAX_Z
                ):
                    raise RuntimeError("Altitude safety envelope exceeded.")

                displacement_1 = float(
                    np.linalg.norm(p1[:2] - initial_1[:2])
                )
                displacement_2 = float(
                    np.linalg.norm(p2[:2] - initial_2[:2])
                )

                if max(displacement_1, displacement_2) > MAX_DISPLACEMENT:
                    raise RuntimeError(
                        "A drone moved too far from its initial hover position."
                    )

                delta = p2[:2] - p1[:2]
                distance = float(np.linalg.norm(delta))

                if distance <= CRITICAL_DISTANCE:
                    raise RuntimeError(
                        f"Critical separation reached: {distance:.3f} m."
                    )

                direction = delta / max(distance, 1e-9)
                error = distance - target_distance
                surprise = calculate_surprisal(distance, target_distance)
                kp = adaptive_gain(surprise)

                v1 = estimate_velocity(p1, previous_1, dt)
                v2 = estimate_velocity(p2, previous_2, dt)
                previous_1 = p1.copy()
                previous_2 = p2.copy()

                # Positive radial velocity means the drones are separating.
                radial_velocity = float(
                    np.dot(v2[:2] - v1[:2], direction)
                )

                # Choose the less-displaced drone as the responder once
                # a displacement is detected. Otherwise both hover.
                moved_1 = displacement_1 > DISPLACEMENT_THRESHOLD
                moved_2 = displacement_2 > DISPLACEMENT_THRESHOLD

                responder = "none"
                cmd1 = np.zeros(2)
                cmd2 = np.zeros(2)

                if moved_1 or moved_2:
                    if moved_1 and not moved_2:
                        responder = "CF2"
                    elif moved_2 and not moved_1:
                        responder = "CF1"
                    else:
                        responder = (
                            "CF1" if displacement_1 < displacement_2
                            else "CF2"
                        )

                    # If separation is too large, move the responder
                    # toward the other drone. If too small, move away.
                    # The damping term opposes fast changes in separation.
                    correction_scalar = (
                        kp * error + KD_RELATIVE * radial_velocity
                    )

                    # Near the minimum distance, prioritize moving apart.
                    if distance < WARNING_DISTANCE:
                        correction_scalar = min(correction_scalar, -0.04)

                    # direction points from CF1 to CF2.
                    if responder == "CF1":
                        # Positive scalar means move CF1 toward CF2.
                        cmd1 = limit_vector(
                            correction_scalar * direction,
                            MAX_SPEED,
                        )
                    else:
                        # Negative scalar means move CF2 toward CF1.
                        cmd2 = limit_vector(
                            -correction_scalar * direction,
                            MAX_SPEED,
                        )

                # Keep altitude with zero vertical velocity.
                send_hover_velocity(cf1, cmd1[0], cmd1[1])
                send_hover_velocity(cf2, cmd2[0], cmd2[1])

                elapsed = now - start_time
                writer.writerow([
                    round(elapsed, 3),
                    round(distance, 4),
                    round(error, 4),
                    round(surprise, 4),
                    round(kp, 4),
                    round(displacement_1, 4),
                    round(displacement_2, 4),
                    responder,
                    round(float(cmd1[0]), 4),
                    round(float(cmd1[1]), 4),
                    round(float(cmd2[0]), 4),
                    round(float(cmd2[1]), 4),
                ])
                file.flush()

                if int(elapsed * 2) != int((elapsed - dt) * 2):
                    print(
                        f"t={elapsed:5.1f}s | d={distance:.3f}m | "
                        f"error={error:+.3f}m | S={surprise:.2f} | "
                        f"responder={responder}"
                    )

                time.sleep(CONTROL_DT)

        finally:
            # Request zero horizontal velocity before the flight wrapper
            # performs its configured cleanup/landing.
            try:
                send_hover_velocity(cf1, 0.0, 0.0)
                send_hover_velocity(cf2, 0.0, 0.0)
            except Exception:
                pass

    print(f"Log saved to: {LOG_PATH}")


def main():
    config = load_config()
    fly_safely(config, NUM_DRONES, mission)


if __name__ == "__main__":
    main()
