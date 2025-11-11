"""
plot_sim_log.py — visualize results from your S-curve simulation.

Usage:
    python plot_sim_log.py
Requires:
    pip install matplotlib pandas
"""

import pandas as pd
import matplotlib.pyplot as plt
import os

# --- Config ---
CSV_FILE = "sim_log.csv"

if not os.path.exists(CSV_FILE):
    raise FileNotFoundError(f"File '{CSV_FILE}' not found. Run your simulation first.")

# --- Load data ---
df = pd.read_csv(CSV_FILE)

# convert radians → degrees for readability
df["theta_deg"] = df["theta"] * 180.0 / 3.141592653589793
df["target_deg"] = df["target_rad"] * 180.0 / 3.141592653589793

# --- Plot ---
plt.style.use("seaborn-v0_8-darkgrid")
fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

# 1. Position
axes[0].plot(df["t_s"], df["theta_deg"], label="Theta (deg)", linewidth=2)
axes[0].plot(df["t_s"], df["target_deg"], "--", label="Target (deg)", linewidth=1.5)
axes[0].set_ylabel("Position (°)")
axes[0].legend(loc="best")
axes[0].grid(True, alpha=0.4)

# 2. Velocity
axes[1].plot(df["t_s"], df["omega"], label="Omega (rad/s)", color="tab:orange", linewidth=2)
axes[1].set_ylabel("Velocity (rad/s)")
axes[1].grid(True, alpha=0.4)

# 3. Acceleration
axes[2].plot(df["t_s"], df["accel"], label="Accel (rad/s²)", color="tab:green", linewidth=2)
axes[2].set_ylabel("Accel (rad/s²)")
axes[2].grid(True, alpha=0.4)

# 4. Jerk (new)
if "jerk" in df.columns:
    axes[3].plot(df["t_s"], df["jerk"], label="Jerk (rad/s³)", color="tab:red", linewidth=2)
else:
    axes[3].text(0.5, 0.5, "Column 'jerk' not found in CSV",
                 transform=axes[3].transAxes, ha="center", va="center",
                 fontsize=11, color="red")
axes[3].set_ylabel("Jerk (rad/s³)")
axes[3].set_xlabel("Time (s)")
axes[3].grid(True, alpha=0.4)

# Optional: shaded areas for active execution
active = df["isExec"] == 1
axes[0].fill_between(df["t_s"], df["theta_deg"].min(), df["theta_deg"].max(),
                     where=active, color="tab:blue", alpha=0.1, label="Executing")
axes[0].legend()

plt.tight_layout()
plt.show()
