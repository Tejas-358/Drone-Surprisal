import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv(
    "data/raw/surprisal_distance_crazysim.csv"
)

fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

axes[0].plot(df["time_s"], df["distance_m"])
axes[0].axhline(0.90, linestyle="--")
axes[0].set_ylabel("Distance (m)")
axes[0].set_title("Distance maintenance")
axes[0].grid(True)

axes[1].plot(df["time_s"], df["surprisal"])
axes[1].set_ylabel("Surprisal")
axes[1].grid(True)

axes[2].plot(df["time_s"], df["kinetic_energy_estimate_J"], label="Kinetic")
axes[2].plot(df["time_s"], df["virtual_distance_potential_proxy"], label="Virtual potential")
axes[2].plot(df["time_s"], df["mechanical_energy_proxy"], label="Total")
axes[2].set_ylabel("Energy (J)")
axes[2].set_xlabel("Time (s)")
axes[2].legend()
axes[2].grid(True)

plt.tight_layout()
plt.show()