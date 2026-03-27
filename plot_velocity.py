#!/usr/bin/env python3
"""
plot_velocity.py — Visualize velocity control simulation output.

Usage:
    python3 plot_velocity.py test/vel_step.csv
    python3 plot_velocity.py test/vel_step.csv test/vel_reverse.csv   (overlay multiple)
    python3 plot_velocity.py test/vel_*.csv                            (all velocity CSVs)

Reads CSV columns: time_s, position, velocity, acceleration, jerk, demand, cmd_vel
Produces a 4-panel plot: position, velocity (with demand overlay), acceleration, jerk vs time.
Demand changes are shown as vertical dashed lines.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from pathlib import Path

RAD2DEG = 180.0 / np.pi
DATA_DIR = Path("./test")


def resolve_paths(input_paths):
    resolved = []
    for p in input_paths:
        path = Path(p)
        if path.exists():
            resolved.append(str(path))
            continue
        test_path = DATA_DIR / p
        if test_path.exists():
            resolved.append(str(test_path))
            continue
    return resolved


def load_csv(path):
    """Load simulation CSV into a dict of numpy arrays."""
    return np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding='utf-8')


def find_demand_changes(data):
    """Return list of (time, demand_deg_per_s) where demand changes."""
    demand = data['demand']
    t = data['time_s']
    changes = []
    prev = demand[0]
    for i in range(1, len(demand)):
        if demand[i] != prev:
            changes.append((t[i], demand[i] * RAD2DEG))
            prev = demand[i]
    if demand[0] != 0.0:
        changes.insert(0, (t[0], demand[0] * RAD2DEG))
    return changes


def shade_accel_regions(ax, data):
    """Shade background based on whether motor is accelerating, cruising, or decelerating."""
    acc = data['acceleration']
    t = data['time_s']
    threshold = 0.05  # rad/s^2

    i = 0
    while i < len(acc):
        if acc[i] > threshold:
            state = 'accel'
            color = '#2196F3'
        elif acc[i] < -threshold:
            state = 'decel'
            color = '#FF9800'
        else:
            state = 'coast'
            color = '#4CAF50'

        start = t[i]
        j = i
        while j < len(acc):
            if state == 'accel' and acc[j] <= threshold:
                break
            elif state == 'decel' and acc[j] >= -threshold:
                break
            elif state == 'coast' and (acc[j] > threshold or acc[j] < -threshold):
                break
            j += 1

        end = t[j - 1] if j < len(t) else t[-1]
        ax.axvspan(start, end, alpha=0.06, color=color, linewidth=0)
        i = j


def plot_single(path):
    """Plot a single velocity CSV file as a 4-panel figure."""
    data = load_csv(path)
    name = os.path.splitext(os.path.basename(path))[0]

    t       = data['time_s']
    pos     = data['position'] * RAD2DEG
    vel     = data['velocity'] * RAD2DEG
    acc     = data['acceleration'] * RAD2DEG
    jrk     = data['jerk'] * RAD2DEG
    demand  = data['demand'] * RAD2DEG
    cmd_vel = data['cmd_vel'] * RAD2DEG

    demand_changes = find_demand_changes(data)

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f'Velocity Control: {name}', fontsize=14, fontweight='bold')

    # --- Panel 1: Position ---
    ax = axes[0]
    shade_accel_regions(ax, data)
    ax.plot(t, pos, color='#1565C0', linewidth=0.8, label='Position')
    ax.set_ylabel('Position [deg]', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=8)
    ax.legend(loc='upper right', fontsize=8)

    # --- Panel 2: Velocity with demand overlay ---
    ax = axes[1]
    shade_accel_regions(ax, data)
    ax.plot(t, vel, color='#2E7D32', linewidth=0.8, label='Actual velocity')
    ax.plot(t, demand, color='red', linewidth=1.0, linestyle='--', alpha=0.7, label='Demand')
    ax.set_ylabel('Velocity [deg/s]', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=8)
    ax.legend(loc='upper right', fontsize=8)

    # --- Panel 3: Acceleration ---
    ax = axes[2]
    shade_accel_regions(ax, data)
    ax.plot(t, acc, color='#E65100', linewidth=0.8)
    ax.set_ylabel('Acceleration [deg/s²]', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=8)
    ax.axhline(0, color='black', linewidth=0.3, alpha=0.5)

    # --- Panel 4: Jerk ---
    ax = axes[3]
    shade_accel_regions(ax, data)
    ax.plot(t, jrk, color='#6A1B9A', linewidth=0.8)
    ax.set_ylabel('Jerk [deg/s³]', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=8)
    ax.axhline(0, color='black', linewidth=0.3, alpha=0.5)

    # Demand change markers on all panels
    for ax in axes:
        for tc, demand_deg in demand_changes:
            ax.axvline(tc, color='red', linestyle='--', alpha=0.4, linewidth=0.7)

    axes[-1].set_xlabel('Time [s]', fontsize=9)

    # Legend at bottom
    legend_patches = [
        Patch(facecolor='#2196F3', alpha=0.2, label='Accelerating'),
        Patch(facecolor='#4CAF50', alpha=0.2, label='Coasting'),
        Patch(facecolor='#FF9800', alpha=0.2, label='Decelerating'),
    ]
    fig.legend(handles=legend_patches, loc='lower center',
               ncol=3, fontsize=7, frameon=True, fancybox=True)

    plt.tight_layout(rect=[0, 0.05, 1, 0.96])

    outpath = DATA_DIR / f"{name}.png"
    fig.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f'  Saved: {outpath}')
    return fig


def plot_overlay(paths):
    """Overlay multiple velocity CSVs on the same 4-panel figure."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Velocity Control — Overlay', fontsize=14, fontweight='bold')

    labels = ['Position [deg]', 'Velocity [deg/s]', 'Acceleration [deg/s²]', 'Jerk [deg/s³]']
    cmap = plt.cm.tab10

    for idx, path in enumerate(paths):
        data = load_csv(path)
        name = os.path.splitext(os.path.basename(path))[0]
        color = cmap(idx % 10)

        t = data['time_s']
        signals = [
            data['position'] * RAD2DEG,
            data['velocity'] * RAD2DEG,
            data['acceleration'] * RAD2DEG,
            data['jerk'] * RAD2DEG,
        ]

        for i, ax in enumerate(axes):
            ax.plot(t, signals[i], color=color, linewidth=0.8,
                    label=name if i == 0 else None)

    for i, ax in enumerate(axes):
        ax.set_ylabel(labels[i], fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    axes[0].legend(loc='upper right', fontsize=7)
    axes[-1].set_xlabel('Time [s]', fontsize=9)

    plt.tight_layout()

    outpath = DATA_DIR / 'vel_overlay.png'
    fig.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f'  Saved: {outpath}')
    return fig


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_velocity.py <csv_file> [csv_file2 ...]")
        print("       python3 plot_velocity.py test/vel_*.csv")
        sys.exit(1)

    paths = resolve_paths(sys.argv[1:])
    paths = [p for p in paths if p.endswith('.csv')]

    # Skip S-curve sim files — only plot velocity CSVs
    paths = [p for p in paths if not os.path.basename(p).startswith('sim_')]

    if not paths:
        print("No valid CSV files found.")
        sys.exit(1)

    print(f"Plotting {len(paths)} file(s)\n")

    for path in paths:
        print(f"  Processing: {path}")
        plot_single(path)

    if len(paths) > 1:
        print(f"\n  Creating overlay of all {len(paths)} files...")
        plot_overlay(paths)

    plt.show()


if __name__ == '__main__':
    main()