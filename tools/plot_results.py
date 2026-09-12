#!/usr/bin/env python3
"""Create publication-ready overview plots from exported experiment CSVs."""

import argparse
from pathlib import Path

import matplotlib
import numpy as np

from experiment_metrics import load_csv_directory


matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('analysis_directory', type=Path)
    parser.add_argument('--output', type=Path, default=None)
    arguments = parser.parse_args()
    diagnostics, states, controls = load_csv_directory(
        arguments.analysis_directory
    )
    output = arguments.output or arguments.analysis_directory / 'overview.png'

    figure, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    x_reference = np.linspace(0.0, 48.0, 961)
    y_reference = 1.2 * np.sin(2.0 * np.pi * x_reference / 24.0)
    axes[0, 0].plot(x_reference, y_reference, '--', color='0.45', label='road')
    axes[0, 0].plot(states[:, 1], states[:, 2], color='#0072B2', label='MRT path')
    obstacle_specs = (
        (14.0, -5.0, 0.0, 0.30, 0.55, '#D55E00'),
        (34.0, 6.0, 0.0, -0.12, 0.55, '#CC79A7'),
    )
    for index, (x0, y0, vx, vy, radius, color) in enumerate(obstacle_specs):
        obstacle_x = x0 + vx * states[:, 0]
        obstacle_y = y0 + vy * states[:, 0]
        axes[0, 0].plot(
            obstacle_x,
            obstacle_y,
            ':',
            color=color,
            label='obstacle tracks' if index == 0 else None,
        )
        distance = np.hypot(states[:, 1] - obstacle_x, states[:, 2] - obstacle_y)
        encounter = int(np.argmin(distance))
        circle = plt.Circle(
            (obstacle_x[encounter], obstacle_y[encounter]),
            radius,
            facecolor=color,
            edgecolor=color,
            alpha=0.35,
        )
        axes[0, 0].add_patch(circle)
    axes[0, 0].set(xlabel='x [m]', ylabel='y [m]', title='Planar trajectory')
    axes[0, 0].axis('equal')
    axes[0, 0].legend()

    axes[0, 1].plot(diagnostics[:, 0], diagnostics[:, 1], label='MRT')
    axes[0, 1].plot(diagnostics[:, 0], diagnostics[:, 2], label='ROV')
    axes[0, 1].axhline(0.0, color='black', linewidth=0.8)
    axes[0, 1].set(
        xlabel='time [s]',
        ylabel='physical margin [m²]',
        title='Safety margins (critical range)',
        ylim=(-1.0, 20.0),
    )
    axes[0, 1].legend()

    axes[1, 0].plot(diagnostics[:, 0], diagnostics[:, 4], color='#D55E00')
    axes[1, 0].axhline(0.30, color='black', linestyle='--', label='deadline')
    axes[1, 0].set(
        xlabel='time [s]', ylabel='solve time [s]', title='NMPC computation'
    )
    axes[1, 0].legend()

    axes[1, 1].plot(controls[:, 0], controls[:, 1], label='acceleration [m/s²]')
    axes[1, 1].plot(
        controls[:, 0], np.degrees(controls[:, 2]), label='steering rate [deg/s]'
    )
    axes[1, 1].set(xlabel='time [s]', title='Applied controls')
    axes[1, 1].legend()
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
    figure.suptitle('ROV-MRT opposite-side dynamic-obstacle experiment')
    figure.savefig(output, dpi=180)
    print(output)


if __name__ == '__main__':
    main()
