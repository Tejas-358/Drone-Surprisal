
"""
surprisal_euler_lagrange_sim.py

Standalone two-drone planar simulation:
- Euler-Lagrange-inspired dynamics
- Fixed-gain vs surprisal-adaptive distance controller
- External push disturbance
- CSV logging and comparison plots

Run:
    python -m src.simulation.surprisal_euler_lagrange_sim

This is a simplified point-mass simulation, NOT CrazySim/Gazebo.
"""

from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# 1. EXPERIMENT CONFIGURATION
# ============================================================

DT = 0.01
DURATION = 12.0

DESIRED_DISTANCE = 0.90       # metres
INITIAL_DISTANCE = 0.90       # metres

# External disturbance pushes drone 1 toward drone 2.
DISTURBANCE_TIME = 3.0        # seconds
DISTURBANCE_DURATION = 0.18   # seconds
DISTURBANCE_FORCE = 0.12      # Newtons

MASS_1 = 0.035                # illustrative effective mass (kg)
MASS_2 = 0.035
DAMPING = 0.10                # N s/m

# Distance potential: V = 0.5 * k * (d - d*)^2
BASE_STIFFNESS = 0.80         # N/m
ADAPTIVE_GAIN = 0.65
SURPRISE_SIGMA = 0.15         # metres
MAX_STIFFNESS = 3.0           # N/m

MAX_CONTROL_FORCE = 0.18      # Newtons
MAX_SPEED = 1.0               # m/s

OUTPUT_DIR = Path("data/simulation/euler_lagrange")


# ============================================================
# 2. HELPER FUNCTIONS
# ============================================================

def clip_vector(vector, max_magnitude):
    """Limit vector magnitude."""
    magnitude = np.linalg.norm(vector)

    if magnitude > max_magnitude and magnitude > 0:
        return vector * (max_magnitude / magnitude)

    return vector


def calculate_surprisal(distance, desired_distance):
    """
    Gaussian distance-error surprisal, omitting the constant term.

    S = 0.5 * ((distance - desired_distance) / sigma)^2
    """
    error = distance - desired_distance

    return 0.5 * (error / SURPRISE_SIGMA) ** 2


def calculate_stiffness(surprisal, controller):
    """Return fixed or surprisal-adaptive virtual stiffness."""

    if controller == "fixed":
        return BASE_STIFFNESS

    if controller == "surprisal_adaptive":
        stiffness = BASE_STIFFNESS * (
            1.0 + ADAPTIVE_GAIN * surprisal
        )

        return min(stiffness, MAX_STIFFNESS)

    raise ValueError(f"Unknown controller: {controller}")


# ============================================================
# 3. EULER-LAGRANGE-INSPIRED SIMULATION
# ============================================================

def run_simulation(controller):
    """
    Simulate two planar point masses.

    Generalized coordinates:
        q1 = [x1, y1]
        q2 = [x2, y2]

    Kinetic energy:
        T = 0.5*m1*|v1|^2 + 0.5*m2*|v2|^2

    Virtual distance potential:
        V = 0.5*k*(distance - desired_distance)^2

    Dynamics:
        m*q_ddot = -grad(V) + external_force - damping*velocity
    """

    times = np.arange(0.0, DURATION + DT / 2, DT)

    q1 = np.array(
        [-INITIAL_DISTANCE / 2, 0.0],
        dtype=float,
    )
    q2 = np.array(
        [INITIAL_DISTANCE / 2, 0.0],
        dtype=float,
    )

    v1 = np.zeros(2)
    v2 = np.zeros(2)

    logs = []

    for t in times:

        # ----------------------------------------------------
        # Measure relative distance
        # ----------------------------------------------------

        relative_position = q2 - q1
        distance = max(
            np.linalg.norm(relative_position),
            1e-9,
        )

        direction = relative_position / distance

        distance_error = distance - DESIRED_DISTANCE

        surprisal = calculate_surprisal(
            distance,
            DESIRED_DISTANCE,
        )

        stiffness = calculate_stiffness(
            surprisal,
            controller,
        )

        # ----------------------------------------------------
        # Euler-Lagrange distance force
        # ----------------------------------------------------

        # V = 0.5*k*(d - d*)^2
        #
        # Drone 1 force:
        #     F1 = +k*(d-d*)*direction
        #
        # Drone 2 force:
        #     F2 = -k*(d-d*)*direction
        #
        # The forces oppose the distance error.

        force_magnitude = stiffness * distance_error

        force1 = force_magnitude * direction
        force2 = -force_magnitude * direction

        force1 = clip_vector(
            force1,
            MAX_CONTROL_FORCE,
        )
        force2 = clip_vector(
            force2,
            MAX_CONTROL_FORCE,
        )

        # ----------------------------------------------------
        # External disturbance
        # ----------------------------------------------------

        external_force1 = np.zeros(2)

        if (
            DISTURBANCE_TIME
            <= t
            < DISTURBANCE_TIME + DISTURBANCE_DURATION
        ):
            # Push drone 1 toward drone 2.
            external_force1 = DISTURBANCE_FORCE * direction

        # ----------------------------------------------------
        # Euler-Lagrange-inspired equations of motion
        # ----------------------------------------------------

        acceleration1 = (
            force1
            + external_force1
            - DAMPING * v1
        ) / MASS_1

        acceleration2 = (
            force2
            - DAMPING * v2
        ) / MASS_2

        # Semi-implicit Euler integration.
        v1 = clip_vector(
            v1 + acceleration1 * DT,
            MAX_SPEED,
        )
        v2 = clip_vector(
            v2 + acceleration2 * DT,
            MAX_SPEED,
        )

        q1 = q1 + v1 * DT
        q2 = q2 + v2 * DT

        # ----------------------------------------------------
        # Energies and metrics after integration
        # ----------------------------------------------------

        new_relative_position = q2 - q1

        new_distance = max(
            np.linalg.norm(new_relative_position),
            1e-9,
        )

        new_error = new_distance - DESIRED_DISTANCE

        new_surprisal = calculate_surprisal(
            new_distance,
            DESIRED_DISTANCE,
        )

        new_stiffness = calculate_stiffness(
            new_surprisal,
            controller,
        )

        kinetic_energy1 = (
            0.5 * MASS_1 * np.dot(v1, v1)
        )

        kinetic_energy2 = (
            0.5 * MASS_2 * np.dot(v2, v2)
        )

        kinetic_energy_total = (
            kinetic_energy1 + kinetic_energy2
        )

        potential_energy = (
            0.5 * new_stiffness * new_error**2
        )

        mechanical_energy = (
            kinetic_energy_total + potential_energy
        )

        external_power = np.dot(
            external_force1,
            v1,
        )

        logs.append({
            "time_s": t,
            "controller": controller,

            "x1_m": q1[0],
            "y1_m": q1[1],
            "x2_m": q2[0],
            "y2_m": q2[1],

            "vx1_m_s": v1[0],
            "vy1_m_s": v1[1],
            "vx2_m_s": v2[0],
            "vy2_m_s": v2[1],

            "distance_m": new_distance,
            "desired_distance_m": DESIRED_DISTANCE,
            "distance_error_m": new_error,

            "surprisal": new_surprisal,
            "stiffness_N_m": new_stiffness,

            "force1_N": np.linalg.norm(force1),
            "force2_N": np.linalg.norm(force2),
            "external_force1_N": np.linalg.norm(
                external_force1
            ),
            "external_power_W": external_power,

            "kinetic_energy1_J": kinetic_energy1,
            "kinetic_energy2_J": kinetic_energy2,
            "kinetic_energy_total_J": kinetic_energy_total,

            "potential_energy_J": potential_energy,
            "mechanical_energy_J": mechanical_energy,
        })

    return pd.DataFrame(logs)


# ============================================================
# 4. LOGGING
# ============================================================

def save_logs(fixed_df, adaptive_df):
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    combined = pd.concat(
        [fixed_df, adaptive_df],
        ignore_index=True,
    )

    csv_path = OUTPUT_DIR / "simulation_log.csv"

    combined.to_csv(
        csv_path,
        index=False,
    )

    print(f"\nCSV log saved to: {csv_path.resolve()}")

    return combined


# ============================================================
# 5. PLOTTING
# ============================================================

def plot_distance(df):
    fig, ax = plt.subplots()

    for controller, group in df.groupby("controller"):
        ax.plot(
            group["time_s"],
            group["distance_m"],
            label=controller,
        )

    ax.axhline(
        DESIRED_DISTANCE,
        linestyle="--",
        label="desired distance",
    )

    ax.axvline(
        DISTURBANCE_TIME,
        linestyle=":",
        label="push begins",
    )

    ax.axvline(
        DISTURBANCE_TIME + DISTURBANCE_DURATION,
        linestyle=":",
        label="push ends",
    )

    ax.set(
        xlabel="Time (s)",
        ylabel="Distance (m)",
        title="Inter-drone distance",
    )

    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    fig.savefig(
        OUTPUT_DIR / "distance_comparison.png",
        dpi=160,
    )


def plot_surprisal(df):
    fig, ax = plt.subplots()

    for controller, group in df.groupby("controller"):
        ax.plot(
            group["time_s"],
            group["surprisal"],
            label=controller,
        )

    ax.axvline(
        DISTURBANCE_TIME,
        linestyle=":",
    )

    ax.set(
        xlabel="Time (s)",
        ylabel="Surprisal",
        title="Surprisal response to disturbance",
    )

    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    fig.savefig(
        OUTPUT_DIR / "surprisal_comparison.png",
        dpi=160,
    )


def plot_energies(df):
    group = df[
        df["controller"] == "surprisal_adaptive"
    ]

    fig, ax = plt.subplots()

    ax.plot(
        group["time_s"],
        group["kinetic_energy_total_J"],
        label="Kinetic energy",
    )

    ax.plot(
        group["time_s"],
        group["potential_energy_J"],
        label="Virtual potential energy",
    )

    ax.plot(
        group["time_s"],
        group["mechanical_energy_J"],
        label="Mechanical energy T + V",
    )

    ax.axvline(
        DISTURBANCE_TIME,
        linestyle=":",
        label="Push begins",
    )

    ax.set(
        xlabel="Time (s)",
        ylabel="Energy (J)",
        title="Energy components: adaptive controller",
    )

    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    fig.savefig(
        OUTPUT_DIR / "energy_components.png",
        dpi=160,
    )


def plot_stiffness(df):
    fig, ax = plt.subplots()

    for controller, group in df.groupby("controller"):
        ax.plot(
            group["time_s"],
            group["stiffness_N_m"],
            label=controller,
        )

    ax.axvline(
        DISTURBANCE_TIME,
        linestyle=":",
    )

    ax.set(
        xlabel="Time (s)",
        ylabel="Stiffness (N/m)",
        title="Surprisal-adaptive stiffness",
    )

    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    fig.savefig(
        OUTPUT_DIR / "stiffness_comparison.png",
        dpi=160,
    )


def plot_trajectories(df):
    fig, ax = plt.subplots()

    for controller, group in df.groupby("controller"):
        ax.plot(
            group["x1_m"],
            group["y1_m"],
            label=f"{controller}: drone 1",
        )

        ax.plot(
            group["x2_m"],
            group["y2_m"],
            linestyle="--",
            label=f"{controller}: drone 2",
        )

    ax.set(
        xlabel="X position (m)",
        ylabel="Y position (m)",
        title="Simulated drone trajectories",
    )

    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    fig.savefig(
        OUTPUT_DIR / "trajectories.png",
        dpi=160,
    )


# ============================================================
# 6. PERFORMANCE SUMMARY
# ============================================================

def print_summary(df):
    print("\n" + "=" * 65)
    print("EXPERIMENT SUMMARY")
    print("=" * 65)

    for controller, group in df.groupby("controller"):
        final_window = group[
            group["time_s"] >= DURATION - 2.0
        ]

        print(f"\nController: {controller}")

        print(
            "Minimum distance: "
            f"{group['distance_m'].min():.3f} m"
        )

        print(
            "Maximum absolute distance error: "
            f"{group['distance_error_m'].abs().max():.3f} m"
        )

        print(
            "Peak surprisal: "
            f"{group['surprisal'].max():.3f}"
        )

        print(
            "Peak kinetic energy: "
            f"{group['kinetic_energy_total_J'].max():.6f} J"
        )

        print(
            "Peak potential energy: "
            f"{group['potential_energy_J'].max():.6f} J"
        )

        print(
            "Mean absolute error during final 2 seconds: "
            f"{final_window['distance_error_m'].abs().mean():.4f} m"
        )

        print(
            "Final distance: "
            f"{group['distance_m'].iloc[-1]:.3f} m"
        )


# ============================================================
# 7. MAIN
# ============================================================

def main():
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    print("Running fixed-gain controller...")
    fixed_df = run_simulation("fixed")

    print("Running surprisal-adaptive controller...")
    adaptive_df = run_simulation("surprisal_adaptive")

    combined = save_logs(
        fixed_df,
        adaptive_df,
    )

    plot_distance(combined)
    plot_surprisal(combined)
    plot_energies(combined)
    plot_stiffness(combined)
    plot_trajectories(combined)

    print_summary(combined)

    print("\nPlots saved in:")
    print(OUTPUT_DIR.resolve())


if __name__ == "__main__":
    main()
