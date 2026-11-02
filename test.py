
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ==========================================
# 1. PARAMETERS
# ==========================================

N = 3                       # Number of drones

m = 1.0                     # Virtual mass
k = 2.0                     # Spring stiffness
c = 1.5                     # Damping coefficient

kt = 0.8                    # Target attraction stiffness
ct = 1.5                    # Target damping

d_desired = 0.6             # Desired distance between drones (m)

dt = 0.01                   # Simulation timestep
simulation_time = 20
steps = int(simulation_time / dt)

target = np.array([3.0, 3.0])


# ==========================================
# 2. INITIAL POSITIONS AND VELOCITIES
# ==========================================

# Random initial positions (x, y)
q = np.random.uniform(-3, 3, (N, 2))

# Initially stationary drones
v = np.zeros((N, 2))

# Store trajectories
trajectory = np.zeros((steps, N, 2))

# Store distances between drones
distances = np.zeros((steps, 3))


# ==========================================
# 3. SIMULATION
# ==========================================

for t in range(steps):

    # --------------------------------------
    # A. Calculate formation centre
    # --------------------------------------

    center = np.mean(q, axis=0)
    center_velocity = np.mean(v, axis=0)

    # --------------------------------------
    # B. Initialize forces
    # --------------------------------------

    F = np.zeros((N, 2))

    # --------------------------------------
    # C. Spring and damping forces
    # --------------------------------------

    for i in range(N):

        for j in range(i + 1, N):

            # Relative position
            relative_position = q[i] - q[j]

            # Distance between drones
            distance = np.linalg.norm(relative_position)

            # Avoid division by zero
            if distance < 1e-6:
                continue

            # Unit direction vector
            direction = relative_position / distance

            # Spring extension/compression
            error = distance - d_desired

            # Spring force
            F_spring = -k * error * direction

            # Relative velocity
            relative_velocity = v[i] - v[j]

            # Damping force
            F_damping = -c * relative_velocity

            # Total interaction force
            F_pair = F_spring + F_damping

            # Newton's third law:
            # Equal and opposite forces
            F[i] += F_pair
            F[j] -= F_pair

    # --------------------------------------
    # D. Target attraction and damping
    # --------------------------------------

    for i in range(N):

        F_target = -(kt / N) * (center - target)

        F_target_damping = -(ct / N) * center_velocity

        F[i] += F_target + F_target_damping

    # --------------------------------------
    # E. Lagrangian equations of motion
    # --------------------------------------

    # m * acceleration = total force

    acceleration = F / m

    # --------------------------------------
    # F. Update velocity and position
    # --------------------------------------

    # Semi-implicit Euler integration

    v = v + acceleration * dt

    q = q + v * dt

    # --------------------------------------
    # G. Store simulation data
    # --------------------------------------

    trajectory[t] = q

    distances[t, 0] = np.linalg.norm(q[0] - q[1])
    distances[t, 1] = np.linalg.norm(q[1] - q[2])
    distances[t, 2] = np.linalg.norm(q[2] - q[0])


# ==========================================
# 4. PRINT FINAL RESULTS
# ==========================================

print("\nFinal drone positions:")

for i in range(N):
    print(f"Drone {i+1}: {q[i]}")

print("\nFinal distances:")

print("D1-D2:", distances[-1, 0])
print("D2-D3:", distances[-1, 1])
print("D3-D1:", distances[-1, 2])

print("\nDesired distance:", d_desired)


# ==========================================
# 5. ANIMATE DRONE MOVEMENT
# ==========================================

fig, ax = plt.subplots(figsize=(8, 7))

ax.set_xlim(-2, 5)
ax.set_ylim(-2, 5)

ax.set_xlabel("X position (m)")
ax.set_ylabel("Y position (m)")
ax.set_title("Lagrangian Multi-Drone Formation Control")

ax.grid(True)

# Drone markers
colors = ["red", "blue", "green"]

drone_markers = []

for i in range(N):

    marker, = ax.plot([], [], "o", color=colors[i],
                      markersize=12, label=f"Drone {i+1}")

    drone_markers.append(marker)

# Formation connections
formation_lines = []

for i, j in [(0, 1), (1, 2), (2, 0)]:

    line, = ax.plot([], [], "k--", linewidth=1.5)

    formation_lines.append(line)

# Trajectories
trajectory_lines = []

for i in range(N):

    line, = ax.plot([], [], color=colors[i], alpha=0.4)

    trajectory_lines.append(line)

# Target marker
ax.plot(target[0], target[1], "k*", markersize=18,
        label="Target")

# Formation centre
center_marker, = ax.plot([], [], "ko", markersize=6,
                         label="Formation centre")

ax.legend()


def update(frame):

    positions = trajectory[frame]

    # Update drone positions
    for i in range(N):

        drone_markers[i].set_data(
            [positions[i, 0]],
            [positions[i, 1]]
        )

        # Draw trajectories
        trajectory_lines[i].set_data(
            trajectory[:frame+1, i, 0],
            trajectory[:frame+1, i, 1]
        )

    # Draw triangular formation
    pairs = [(0, 1), (1, 2), (2, 0)]

    for index, (i, j) in enumerate(pairs):

        formation_lines[index].set_data(
            [positions[i, 0], positions[j, 0]],
            [positions[i, 1], positions[j, 1]]
        )

    # Update formation centre
    center = np.mean(positions, axis=0)

    center_marker.set_data([center[0]], [center[1]])

    ax.set_title(
        f"Lagrangian Formation Control | Time: {frame*dt:.2f}s"
    )

    return (
        drone_markers
        + formation_lines
        + trajectory_lines
        + [center_marker]
    )


animation = FuncAnimation(
    fig,
    update,
    frames=range(0, steps, 5),
    interval=20,
    blit=False
)

plt.show()


# ==========================================
# 6. PLOT DISTANCE CONVERGENCE
# ==========================================

time = np.arange(steps) * dt

plt.figure(figsize=(10, 5))

plt.plot(time, distances[:, 0], label="Drone 1 - Drone 2")
plt.plot(time, distances[:, 1], label="Drone 2 - Drone 3")
plt.plot(time, distances[:, 2], label="Drone 3 - Drone 1")

plt.axhline(
    y=d_desired,
    color="black",
    linestyle="--",
    label="Desired distance"
)

plt.xlabel("Time (seconds)")
plt.ylabel("Distance (metres)")

plt.title("Drone Formation Distance Convergence")

plt.legend()
plt.grid(True)
plt.show()
