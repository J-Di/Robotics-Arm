#!/usr/bin/env python3
"""
plot_trajectory.py — Visualize S-Curve trajectory simulation output.

Usage:
    python3 plot_trajectory.py sim_simple.csv
    python3 plot_trajectory.py sim_simple.csv sim_extend.csv   (overlay multiple)
    python3 plot_trajectory.py *.csv                            (all CSVs)

Reads CSV columns: time_s, position, velocity, acceleration, jerk, phase, setpoint, dir
Produces a 4-panel plot: position, velocity, acceleration, jerk vs time.
Setpoint changes are shown as vertical dashed lines.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import matplotlib.ticker as ticker
from pathlib import Path

# Phase color map — matches the 7-phase S-curve
PHASE_COLORS = {
    0: '#e0e0e0',   # idle (grey)
    1: '#2196F3',   # Phase I:   +jerk  (blue)
    2: '#42A5F5',   # Phase II:  0 jerk (light blue)
    3: '#64B5F6',   # Phase III: -jerk  (lighter blue)
    4: '#4CAF50',   # Phase IV:  cruise (green)
    5: '#FF9800',   # Phase V:   -jerk  (orange)
    6: '#FFA726',   # Phase VI:  0 jerk (light orange)
    7: '#FFB74D',   # Phase VII: +jerk  (lighter orange)
}

PHASE_LABELS = {
    0: 'Idle',
    1: 'I: +j (accel up)',
    2: 'II: 0j (const accel)',
    3: 'III: -j (accel down)',
    4: 'IV: cruise',
    5: 'V: -j (decel up)',
    6: 'VI: 0j (const decel)',
    7: 'VII: +j (decel down)',
}

RAD2DEG = 180.0 / np.pi



DATA_DIR = Path("./test")

def resolve_paths(input_paths):
    resolved = []

    for p in input_paths:
        path = Path(p)

        # Case 1: exact path exists
        if path.exists():
            resolved.append(str(path))
            continue

        # Case 2: try inside ./test/
        test_path = DATA_DIR / p
        if test_path.exists():
            resolved.append(str(test_path))
            continue

    return resolved


def load_csv(path):
    """Load simulation CSV into a dict of numpy arrays."""
    data = np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding='utf-8')
    return data


def find_setpoint_changes(data):
    """Return list of (time, setpoint_deg) where setpoint changes."""
    sp = data['setpoint']
    t  = data['time_s']
    changes = []
    prev = sp[0]
    for i in range(1, len(sp)):
        if sp[i] != prev:
            changes.append((t[i], sp[i] * RAD2DEG))
            prev = sp[i]
    # Also include initial if non-zero
    if sp[0] != 0.0:
        changes.insert(0, (t[0], sp[0] * RAD2DEG))
    return changes


def shade_phases(ax, data):
    """Add light background shading for each phase region."""
    phases = data['phase']
    t = data['time_s']
    
    i = 0
    while i < len(phases):
        p = phases[i]
        start = t[i]
        # Find end of this contiguous phase block
        j = i
        while j < len(phases) and phases[j] == p:
            j += 1
        end = t[j - 1] if j < len(t) else t[-1]
        
        if p in PHASE_COLORS:
            ax.axvspan(start, end, alpha=0.08, color=PHASE_COLORS[p], linewidth=0)
        i = j


def plot_single(path):
    """Plot a single CSV file as a 4-panel figure."""
    data = load_csv(path)
    name = os.path.splitext(os.path.basename(path))[0]
    
    t    = data['time_s']
    pos  = data['position']  * RAD2DEG
    vel  = data['velocity']  * RAD2DEG
    acc  = data['acceleration'] * RAD2DEG
    jrk  = data['jerk']     * RAD2DEG
    sp   = data['setpoint']  * RAD2DEG

    sp_changes = find_setpoint_changes(data)

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f'S-Curve Trajectory: {name}', fontsize=14, fontweight='bold')

    labels = ['Position [deg]', 'Velocity [deg/s]', 'Acceleration [deg/s²]', 'Jerk [deg/s³]']
    signals = [pos, vel, acc, jrk]
    colors = ['#1565C0', '#2E7D32', '#E65100', '#6A1B9A']

    for i, ax in enumerate(axes):
        shade_phases(ax, data)
        ax.plot(t, signals[i], color=colors[i], linewidth=0.8)
        ax.set_ylabel(labels[i], fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

        # Setpoint markers
        for tc, sp_deg in sp_changes:
            ax.axvline(tc, color='red', linestyle='--', alpha=0.5, linewidth=0.7)

        # Show setpoint line on position plot
        if i == 0:
            ax.plot(t, sp, color='red', linestyle=':', linewidth=1.0, alpha=0.7, label='Setpoint')
            ax.legend(loc='upper right', fontsize=8)

    axes[-1].set_xlabel('Time [s]', fontsize=9)

    # Phase legend at bottom
    used_phases = sorted(set(data['phase']))
    legend_patches = [Patch(facecolor=PHASE_COLORS.get(p, '#ccc'), alpha=0.3,
                            label=PHASE_LABELS.get(p, f'Phase {p}'))
                      for p in used_phases if p in PHASE_LABELS]
    if legend_patches:
        fig.legend(handles=legend_patches, loc='lower center', 
                   ncol=min(len(legend_patches), 4), fontsize=7,
                   frameon=True, fancybox=True)

    plt.tight_layout(rect=[0, 0.06, 1, 0.96])

    outpath = DATA_DIR / f"{name}.png"
    fig.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f'  Saved: {outpath}')
    return fig


def plot_overlay(paths):
    """Overlay multiple CSVs on the same 4-panel figure."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('S-Curve Trajectories — Overlay', fontsize=14, fontweight='bold')

    labels = ['Position [deg]', 'Velocity [deg/s]', 'Acceleration [deg/s²]', 'Jerk [deg/s³]']
    cmap = plt.cm.tab10

    for idx, path in enumerate(paths):
        data = load_csv(path)
        name = os.path.splitext(os.path.basename(path))[0]
        color = cmap(idx % 10)

        t    = data['time_s']
        signals = [
            data['position'] * RAD2DEG,
            data['velocity'] * RAD2DEG,
            data['acceleration'] * RAD2DEG,
            data['jerk'] * RAD2DEG,
        ]

        for i, ax in enumerate(axes):
            ax.plot(t, signals[i], color=color, linewidth=0.8, label=name if i == 0 else None)

    for i, ax in enumerate(axes):
        ax.set_ylabel(labels[i], fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    axes[0].legend(loc='upper right', fontsize=7)
    axes[-1].set_xlabel('Time [s]', fontsize=9)

    plt.tight_layout()

    outpath = 'sim_overlay.png'
    fig.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f'  Saved: {outpath}')
    return fig


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_trajectory.py <csv_file> [csv_file2 ...]")
        print("       python3 plot_trajectory.py *.csv")
        sys.exit(1)

    paths = resolve_paths(sys.argv[1:])
    paths = [p for p in paths if p.endswith('.csv')]

    if not paths:
        print("No valid CSV files found.")
        sys.exit(1)

    print(f"Plotting {len(paths)} file(s)...\n")

    # Plot each individually
    for path in paths:
        print(f"  Processing: {path}")
        plot_single(path)

    # If multiple files, also create an overlay
    if len(paths) > 1:
        print(f"\n  Creating overlay of all {len(paths)} files...")
        plot_overlay(paths)

    plt.show()


if __name__ == '__main__':
    main()
