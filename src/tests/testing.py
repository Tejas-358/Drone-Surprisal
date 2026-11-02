
"""
coordinate_navigation_test.py

Two-Crazyflie Lighthouse coordinate navigation.

Run from the Drone-Surprisal repository root:
    python -m src.tests.coordinate_navigation_test --mode real

IMPORTANT:
- Configure two real Crazyflie URIs in your project configuration.
- Verify world-frame position and velocity-command behavior.
- Choose targets close to each drone's measured starting position.
- Maintain adequate physical separation and supervise the flight.
"""

import argparse
import math
import time
import warnings

from src.utils.connection import load_config
from src.utils.flight import fly_safely
from src.utils.safety import watch

warnings.filterwarnings("ignore")


DESIRED_DISTANCE = 0.60
DISTANCE_TOLERANCE = 0.05
MIN_SAFE_SEPARATION = 0.10

CONTROL_PERIOD = 0.1
KP_DISTANCE = 0.25
MAX_SPEED = 0.05

HOVER_SECONDS = 5.0
NAVIGATION_TIMEOUT = 60.0

# Safety limits
MAX_DISTANCE_FROM_START = 4.0
MIN_Z = 0.10
MAX_Z = 0.6
MAX_ABS_XY = 3.0


def horizontal_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def read_position(state):
    try:
        p = (
            float(state["x"]),
            float(state["y"]),
            float(state["z"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("Missing or invalid position telemetry.") from exc

    if not all(math.isfinite(v) for v in p):
        raise RuntimeError(f"Invalid position telemetry: {p}")

    return p


def check_target(target):
    x, y, z = target

    if abs(x) > MAX_ABS_XY or abs(y) > MAX_ABS_XY:
        raise ValueError(f"Target exceeds X/Y bounds: {target}")

    if not MIN_Z <= z <= MAX_Z:
        raise ValueError(
            f"Target altitude must be between {MIN_Z} and {MAX_Z} m: "
            f"{target}"
        )


def check_position(position, start, drone_name, phase):
    x, y, z = position

    if horizontal_distance(start, position) > MAX_DISTANCE_FROM_START:
        raise RuntimeError(
            f"{drone_name}: horizontal displacement limit exceeded "
            f"during {phase}. Start={start}, current={position}"
        )

    if abs(x) > MAX_ABS_XY or abs(y) > MAX_ABS_XY:
        raise RuntimeError(
            f"{drone_name}: world-frame X/Y bound exceeded: {position}"
        )

    if not MIN_Z <= z <= MAX_Z:
        raise RuntimeError(
            f"{drone_name}: altitude outside "
            f"[{MIN_Z:.2f}, {MAX_Z:.2f}] m: {position}"
        )



def send_velocity(scf, velocity):
    vx, vy, vz = velocity
    scf.cf.commander.send_velocity_world_setpoint(
        vx, vy, vz, 0.0
    )


def mission(drones, config, states):
    if len(drones) != 2 or len(states) != 2:
        raise RuntimeError("Exactly two configured drones are required.")

    names = ["CF1", "CF2"]
    starts = [read_position(s) for s in states]

    if DESIRED_DISTANCE <= MIN_SAFE_SEPARATION:
        raise ValueError(
            "DESIRED_DISTANCE must exceed MIN_SAFE_SEPARATION."
        )

    def separation_and_direction(positions):
        # Vector from CF1 to CF2 in the Lighthouse world frame.
        dx = positions[1][0] - positions[0][0]
        dy = positions[1][1] - positions[0][1]
        dz = positions[1][2] - positions[0][2]

        d = math.sqrt(dx * dx + dy * dy + dz * dz)

        if d < 1e-6:
            raise RuntimeError(
                "Drone positions are nearly identical. "
                "Stopping to avoid unsafe commands."
            )

        return d, (dx / d, dy / d, dz / d)

    def check_live_positions(positions, phase):
        for i in range(2):
            check_position(
                positions[i], starts[i], names[i], phase
            )

        d, _ = separation_and_direction(positions)

        if d < MIN_SAFE_SEPARATION:
            # Best-effort immediate stop before fly_safely handles landing.
            for drone in drones:
                drone.cf.commander.send_stop_setpoint()

            raise RuntimeError(
                f"Emergency separation limit reached during {phase}: "
                f"{d:.3f} m. Stopping and requesting safe landing."
            )

        return d

    def distance_controller(positions):
        d, direction = separation_and_direction(positions)

        # Positive error means too far apart.
        # Negative error means too close.
        error = d - DESIRED_DISTANCE

        if abs(error) <= DISTANCE_TOLERANCE:
            return [(0.0, 0.0, 0.0)] * 2

        # Move symmetrically along the line joining the drones.
        speed = min(KP_DISTANCE * abs(error), MAX_SPEED)

        # direction points from CF1 toward CF2.
        if error > 0:
            # Too far apart: approach each other.
            v1 = tuple(speed * u for u in direction)
            v2 = tuple(-speed * u for u in direction)
        else:
            # Too close: move apart.
            v1 = tuple(-speed * u for u in direction)
            v2 = tuple(speed * u for u in direction)

        return [v1, v2]

    print("Initial positions:")
    for i in range(2):
        print(f"{names[i]}: {starts[i]}")

    initial_distance, _ = separation_and_direction(starts)
    print(f"Initial separation: {initial_distance:.3f} m")
    print(f"Desired separation: {DESIRED_DISTANCE:.3f} m")

    if initial_distance < MIN_SAFE_SEPARATION:
        raise RuntimeError(
            "Initial separation is below the emergency threshold. "
            "Do not start the experiment."
        )

    # Confirm both drones are within the configured world bounds.
    for i, p in enumerate(starts):
        if abs(p[0]) > MAX_ABS_XY or abs(p[1]) > MAX_ABS_XY:
            raise RuntimeError(
                f"{names[i]} starts outside the configured XY bounds."
            )

    print("Starting distance-maintenance controller.")
    start_time = time.monotonic()
    last_log = start_time

    while time.monotonic() - start_time < NAVIGATION_TIMEOUT:
        positions = [read_position(s) for s in states]
        d = check_live_positions(positions, "control")

        error = d - DESIRED_DISTANCE

        # Stop both drones when inside the target distance band.
        if abs(error) <= DISTANCE_TOLERANCE:
            for drone in drones:
                drone.cf.commander.send_stop_setpoint()

            print(
                f"Target distance reached: {d:.3f} m "
                f"(error={error:+.3f} m)"
            )
            break

        velocities = distance_controller(positions)

        # Recheck the live positions immediately before sending commands.
        latest = [read_position(s) for s in states]
        check_live_positions(latest, "before command")

        for i in range(2):
            send_velocity(drones[i], velocities[i])

        now = time.monotonic()
        if now - last_log >= 0.5:
            print(
                f"Separation={d:.3f} m | "
                f"error={error:+.3f} m | "
                f"CF1 velocity={velocities[0]} | "
                f"CF2 velocity={velocities[1]}"
            )
            last_log = now

        watch(states, config, CONTROL_PERIOD)

    else:
        raise TimeoutError(
            "Timed out before reaching the desired separation."
        )

    # Maintain the desired distance for a short observation period.
    print(f"Maintaining separation for {HOVER_SECONDS:.1f} seconds...")
    hover_start = time.monotonic()

    while time.monotonic() - hover_start < HOVER_SECONDS:
        positions = [read_position(s) for s in states]
        d = check_live_positions(positions, "distance maintenance")

        velocities = distance_controller(positions)

        latest = [read_position(s) for s in states]
        check_live_positions(latest, "before maintenance command")

        for i in range(2):
            send_velocity(drones[i], velocities[i])

        print(
            f"Current separation={d:.3f} m; "
            f"desired={DESIRED_DISTANCE:.3f} m"
        )

        watch(states, config, CONTROL_PERIOD)

    for drone in drones:
        drone.cf.commander.send_stop_setpoint()

    print("Distance-maintenance test complete; returning for landing.")


def main():
    parser = argparse.ArgumentParser(
        description="Two-Crazyflie coordinate navigation test."
    )
    parser.add_argument(
        "--mode",
        choices=["sim", "real"],
        default="sim",
        help="Override the mode in the project config.",
    )
    args = parser.parse_args()

    config = load_config()
    config["mode"] = args.mode

    # Connect two drones. The configuration must provide two valid URIs.
    fly_safely(config, 2, mission)

    print("Two-drone coordinate navigation test complete.")


if __name__ == "__main__":
    main()

