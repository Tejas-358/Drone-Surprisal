
"""
coordinate_navigation_test.py

Conservative single-Crazyflie Lighthouse navigation test.

Run from the Drone-Surprisal repository root:
    python -m src.tests.coordinate_navigation_test --mode real

Assumptions:
- state["x"], state["y"], state["z"] are in metres.
- Position estimates are expressed in the Lighthouse world frame.
- send_velocity_world_setpoint(vx, vy, vz, yawrate) is supported.
- fly_safely() manages connection, preflight, takeoff and landing.

IMPORTANT:
- Verify coordinate-frame behavior before autonomous flight.
- The horizontal travel limit is measured from the live takeoff position.
- Software guardrails cannot guarantee safety if telemetry or control frames
  are incorrect.
"""

import argparse
import math
import time

from src.utils.connection import load_config
from src.utils.flight import fly_safely
from src.utils.safety import watch


# Target in Lighthouse world coordinates (metres).
# Keep altitude close to the takeoff altitude for the initial test.
TARGET = (0.2, 0.2, 0.4)

# Conservative controller settings.
CONTROL_PERIOD = 0.1
MAX_SPEED = 0.05               # m/s
KP = 0.3
ARRIVAL_TOLERANCE = 0.08       # metres
HOVER_SECONDS = 4.0
NAVIGATION_TIMEOUT = 15.0

# Safety guardrails.
MAX_DISTANCE_FROM_START = 1  # horizontal distance, metres
MIN_Z = 0.10
MAX_Z = 0.50
MAX_ABS_XY = 1.8


def horizontal_distance(a, b):
    """Return horizontal XY distance in metres."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def distance(a, b):
    """Return 3D Euclidean distance in metres."""
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def read_position(state):
    """Return finite (x, y, z) telemetry, or fail safely."""
    try:
        position = (
            float(state["x"]),
            float(state["y"]),
            float(state["z"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(
            "Missing or invalid position telemetry."
        ) from exc

    if not all(math.isfinite(value) for value in position):
        raise RuntimeError(
            f"Invalid position telemetry: {position}"
        )

    return position


def check_target(target):
    """Validate the target against the configured world bounds."""
    x, y, z = target

    if abs(x) > MAX_ABS_XY or abs(y) > MAX_ABS_XY:
        raise ValueError(
            "Target exceeds the configured X/Y guardrails."
        )

    if not MIN_Z <= z <= MAX_Z:
        raise ValueError(
            f"Target altitude must be between {MIN_Z} and {MAX_Z} m."
        )


def check_position(position, start, phase):
    """
    Validate the latest measured position.

    Call this immediately before every movement setpoint.
    """
    x, y, z = position

    if horizontal_distance(start, position) > MAX_DISTANCE_FROM_START:
        raise RuntimeError(
            f"Safety stop during {phase}: horizontal displacement "
            f"from start exceeded {MAX_DISTANCE_FROM_START:.2f} m. "
            f"Start={start}, current={position}"
        )

    if abs(x) > MAX_ABS_XY or abs(y) > MAX_ABS_XY:
        raise RuntimeError(
            f"Safety stop during {phase}: world-frame X/Y bound exceeded. "
            f"Current position={position}"
        )

    if not MIN_Z <= z <= MAX_Z:
        raise RuntimeError(
            f"Safety stop during {phase}: altitude outside "
            f"[{MIN_Z:.2f}, {MAX_Z:.2f}] m. "
            f"Current position={position}"
        )


def send_velocity(scf, vx, vy, vz):
    """Send one velocity setpoint, capped by the 3D speed limit."""
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)

    if speed > MAX_SPEED:
        scale = MAX_SPEED / speed
        vx *= scale
        vy *= scale
        vz *= scale

    scf.cf.commander.send_velocity_world_setpoint(
        vx, vy, vz, 0.0
    )


def mission(drones, config, states):
    """Navigate one drone, hold position briefly, then return to fly_safely."""
    if len(drones) != 1 or len(states) != 1:
        raise RuntimeError(
            "This test is intended for exactly one drone."
        )

    scf = drones[0]
    target = tuple(float(value) for value in TARGET)
    check_target(target)

    # Read the live starting position after takeoff.
    start = read_position(states[0])

    print(f"Starting position: {start}")
    print(f"Target position:   {target}")

    # Check the target's required horizontal displacement before moving.
    requested_horizontal_distance = horizontal_distance(start, target)

    if requested_horizontal_distance > MAX_DISTANCE_FROM_START:
        raise RuntimeError(
            "Target rejected before navigation: its horizontal distance "
            f"from the starting position is "
            f"{requested_horizontal_distance:.3f} m, exceeding the "
            f"{MAX_DISTANCE_FROM_START:.2f} m test limit. "
            "No navigation setpoint was sent."
        )

    # Check that the initial telemetry is within the allowed flight envelope.
    check_position(start, start, "navigation startup")

    print("Navigating using position feedback...")
    start_time = time.monotonic()
    last_log = start_time

    while True:
        now = time.monotonic()
        position = read_position(states[0])

        # Safety checks happen before the target calculation and setpoint.
        check_position(position, start, "navigation")

        error = tuple(
            target_value - current_value
            for target_value, current_value in zip(target, position)
        )
        remaining = distance(position, target)

        if remaining <= ARRIVAL_TOLERANCE:
            print(
                f"Target reached: position={position}, "
                f"error={remaining:.3f} m"
            )
            break

        if now - start_time > NAVIGATION_TIMEOUT:
            raise TimeoutError(
                f"Navigation timed out at {position}; "
                f"target error={remaining:.3f} m."
            )

        # Proportional position controller -> bounded velocity.
        vx, vy, vz = (KP * component for component in error)

        # Recheck the live position immediately before issuing movement.
        latest_position = read_position(states[0])
        check_position(latest_position, start, "before navigation command")

        send_velocity(scf, vx, vy, vz)

        if now - last_log >= 0.5:
            print(
                f"Position=({position[0]:.2f}, "
                f"{position[1]:.2f}, {position[2]:.2f}) | "
                f"error={remaining:.2f} m"
            )
            last_log = now

        # Preserve the project's in-flight safety monitoring.
        watch(states, config, CONTROL_PERIOD)

    print("Holding target position...")
    hover_start = time.monotonic()

    while time.monotonic() - hover_start < HOVER_SECONDS:
        position = read_position(states[0])
        check_position(position, start, "hover")

        error = tuple(
            target_value - current_value
            for target_value, current_value in zip(target, position)
        )
        vx, vy, vz = (KP * component for component in error)

        # Recheck before every hover correction too.
        latest_position = read_position(states[0])
        check_position(latest_position, start, "before hover command")

        send_velocity(scf, vx, vy, vz)
        watch(states, config, CONTROL_PERIOD)

    print("Hover complete. Returning to fly_safely() for landing.")


def main():
    parser = argparse.ArgumentParser(
        description="Conservative single-Crazyflie coordinate navigation test."
    )
    parser.add_argument(
        "--mode",
        choices=["sim", "real"],
        default="real",
        help="Override the mode in the project config.",
    )
    args = parser.parse_args()

    config = load_config()
    config["mode"] = args.mode

    # One drone only: CF1.
    fly_safely(config, 1, mission)
    print("Coordinate navigation test complete.")


if __name__ == "__main__":
    main()
