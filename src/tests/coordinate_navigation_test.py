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


# Targets are (x, y, z), in metres in the Lighthouse world frame.
# EXAMPLES ONLY: adjust these to your measured starting positions.
TARGETS = [
    (0.72, 0.40, 0.2),    # CF1 target
    (0.72, -0.70, 0.2),  # CF2 target
]

CONTROL_PERIOD = 0.1
MAX_SPEED = 0.05
KP = 0.3
ARRIVAL_TOLERANCE = 0.08
HOVER_SECONDS = 2.0
NAVIGATION_TIMEOUT = 45.0

# Safety limits
MAX_DISTANCE_FROM_START = 3.0
MIN_Z = 0.10
MAX_Z = 0.50
MAX_ABS_XY = 3.0

# Minimum allowed separation between drone position estimates.
# This is a software stop threshold, NOT a collision-avoidance guarantee.
MIN_DRONE_SEPARATION = 0.00


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


def check_separation(positions, phase):
    if len(positions) != 2:
        raise RuntimeError("Expected exactly two drone positions.")

    separation = distance(positions[0], positions[1])

    if separation < MIN_DRONE_SEPARATION:
        raise RuntimeError(
            f"Safety stop during {phase}: drone separation is "
            f"{separation:.3f} m, below "
            f"{MIN_DRONE_SEPARATION:.2f} m. "
            "Landing should be managed by fly_safely()."
        )


def bounded_velocity(position, target):
    error = tuple(t - p for p, t in zip(position, target))
    velocity = [KP * e for e in error]

    speed = math.sqrt(sum(v * v for v in velocity))

    if speed > MAX_SPEED:
        scale = MAX_SPEED / speed
        velocity = [v * scale for v in velocity]

    return velocity


def send_velocity(scf, velocity):
    vx, vy, vz = velocity
    scf.cf.commander.send_velocity_world_setpoint(
        vx, vy, vz, 0.0
    )


def mission(drones, config, states):
    if len(drones) != 2 or len(states) != 2:
        raise RuntimeError(
            "This test requires exactly two configured drones."
        )

    if len(TARGETS) != 2:
        raise RuntimeError("Provide one target per drone.")

    targets = [tuple(map(float, t)) for t in TARGETS]

    for target in targets:
        check_target(target)

    # Reject targets that are too close together.
    target_separation = distance(targets[0], targets[1])

    if target_separation < MIN_DRONE_SEPARATION:
        raise ValueError(
            f"Targets are only {target_separation:.3f} m apart. "
            f"Use targets at least {MIN_DRONE_SEPARATION:.2f} m apart."
        )

    names = ["CF1", "CF2"]
    starts = [read_position(s) for s in states]

    for i in range(2):
        print(f"{names[i]} start:  {starts[i]}")
        print(f"{names[i]} target: {targets[i]}")

        if horizontal_distance(starts[i], targets[i]) > MAX_DISTANCE_FROM_START:
            raise RuntimeError(
                f"{names[i]} target is more than "
                f"{MAX_DISTANCE_FROM_START:.2f} m horizontally "
                "from its starting position. No navigation started."
            )

        check_position(starts[i], starts[i], names[i], "startup")

    check_separation(starts, "startup")

    # Check the straight-line target paths for possible conflicts.
    # This is a simple conservative check, not full trajectory planning.
    for alpha in [i / 20 for i in range(21)]:
        predicted = [
            tuple(
                starts[j][k] + alpha * (targets[j][k] - starts[j][k])
                for k in range(3)
            )
            for j in range(2)
        ]
        check_separation(predicted, "planned path check")

    print("Both drones passed startup checks.")
    print("Navigating both drones using position feedback...")

    start_time = time.monotonic()
    last_log = start_time

    while True:
        positions = [read_position(s) for s in states]

        for i in range(2):
            check_position(
                positions[i], starts[i], names[i], "navigation"
            )

        check_separation(positions, "navigation")

        errors = [
            distance(positions[i], targets[i]) for i in range(2)
        ]

        if all(e <= ARRIVAL_TOLERANCE for e in errors):
            for i in range(2):
                print(
                    f"{names[i]} target reached: "
                    f"position={positions[i]}, error={errors[i]:.3f} m"
                )
            break

        if time.monotonic() - start_time > NAVIGATION_TIMEOUT:
            raise TimeoutError(
                "Navigation timed out. "
                f"CF1 error={errors[0]:.3f} m; "
                f"CF2 error={errors[1]:.3f} m."
            )

        # Calculate both commands before sending either one.
        velocities = [
            bounded_velocity(positions[i], targets[i])
            for i in range(2)
        ]

        # Recheck both live positions immediately before movement.
        latest = [read_position(s) for s in states]

        for i in range(2):
            check_position(
                latest[i], starts[i], names[i], "before command"
            )

        check_separation(latest, "before command")

        # Issue both commands in each control cycle.
        for i in range(2):
            if errors[i] > ARRIVAL_TOLERANCE:
                send_velocity(drones[i], velocities[i])
            else:
                # Zero velocity while the other drone finishes.
                send_velocity(drones[i], (0.0, 0.0, 0.0))

        now = time.monotonic()

        if now - last_log >= 0.5:
            print(
                f"CF1={positions[0]} error={errors[0]:.2f} m | "
                f"CF2={positions[1]} error={errors[1]:.2f} m"
            )
            last_log = now

        watch(states, config, CONTROL_PERIOD)

    print(f"Holding both targets for {HOVER_SECONDS:.1f} seconds...")
    hover_start = time.monotonic()

    while time.monotonic() - hover_start < HOVER_SECONDS:
        positions = [read_position(s) for s in states]

        for i in range(2):
            check_position(
                positions[i], starts[i], names[i], "hover"
            )

        check_separation(positions, "hover")

        velocities = [
            bounded_velocity(positions[i], targets[i])
            for i in range(2)
        ]

        latest = [read_position(s) for s in states]

        for i in range(2):
            check_position(
                latest[i], starts[i], names[i], "before hover command"
            )

        check_separation(latest, "before hover command")

        for i in range(2):
            send_velocity(drones[i], velocities[i])

        watch(states, config, CONTROL_PERIOD)

    print("Hover complete. Returning to fly_safely() for landing.")


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
