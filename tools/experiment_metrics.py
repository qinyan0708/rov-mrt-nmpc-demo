"""Metric calculation shared by bag extraction and CSV-only evaluation."""

import json
from pathlib import Path

import numpy as np


def compute_metrics(diagnostics, states, controls, deadline=0.30):
    """Compute scalar experiment metrics from numeric arrays with time columns."""
    solve_time = diagnostics[:, 4]
    active = diagnostics[:, 6] > 0.5
    dt = np.diff(diagnostics[:, 0])
    median_dt = float(np.median(dt)) if dt.size else 0.0
    return {
        'diagnostic_samples': int(diagnostics.shape[0]),
        'state_samples': int(states.shape[0]),
        'control_samples': int(controls.shape[0]),
        'min_mrt_physical_margin_m2': float(np.min(diagnostics[:, 1])),
        'min_rov_physical_margin_m2': float(np.min(diagnostics[:, 2])),
        'max_safety_slack_m2': float(np.max(diagnostics[:, 3])),
        'max_constraint_violation': float(np.max(diagnostics[:, 5])),
        'mean_solve_time_s': float(np.mean(solve_time)),
        'p95_solve_time_s': float(np.percentile(solve_time, 95)),
        'max_solve_time_s': float(np.max(solve_time)),
        'solve_samples_over_deadline': int(np.sum(solve_time > deadline)),
        'deadline_s': float(deadline),
        'avoidance_active_time_s': float(np.sum(active) * median_dt),
        'min_velocity_mps': float(np.min(states[:, 4])),
        'max_velocity_mps': float(np.max(states[:, 4])),
        'max_abs_steering_deg': float(np.degrees(np.max(np.abs(states[:, 5])))),
        'max_abs_acceleration_mps2': float(np.max(np.abs(controls[:, 1]))),
        'max_abs_steering_rate_deg_s': float(
            np.degrees(np.max(np.abs(controls[:, 2])))
        ),
    }


def format_summary(metrics, source):
    """Return a stable human-readable summary used in documentation."""
    return f"""ROV-MRT ROS 2 NMPC experiment summary
============================================
Source: {source}
Diagnostic samples: {metrics['diagnostic_samples']}
State samples:      {metrics['state_samples']}
Control samples:    {metrics['control_samples']}

Min MRT physical margin: {metrics['min_mrt_physical_margin_m2']:.6f} m^2
Min ROV physical margin: {metrics['min_rov_physical_margin_m2']:.6f} m^2
Max safety slack:        {metrics['max_safety_slack_m2']:.6e} m^2
Max constraint violation:{metrics['max_constraint_violation']:.6e}

Mean solve time:         {metrics['mean_solve_time_s']:.6f} s
95th-percentile time:    {metrics['p95_solve_time_s']:.6f} s
Max solve time:          {metrics['max_solve_time_s']:.6f} s
Samples over {metrics['deadline_s']:.2f} s:     {metrics['solve_samples_over_deadline']}

Avoidance-active time:   {metrics['avoidance_active_time_s']:.3f} s
Velocity range:          [{metrics['min_velocity_mps']:.4f}, {metrics['max_velocity_mps']:.4f}] m/s
Max |steering angle|:    {metrics['max_abs_steering_deg']:.3f} deg
Max |acceleration|:      {metrics['max_abs_acceleration_mps2']:.4f} m/s^2
Max |steering rate|:     {metrics['max_abs_steering_rate_deg_s']:.3f} deg/s"""


def write_summary(path, metrics):
    """Write metrics as deterministic, machine-readable JSON."""
    Path(path).write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def load_csv_directory(directory):
    """Load the three standard analysis CSV files."""
    directory = Path(directory)
    return (
        np.loadtxt(directory / 'diagnostics.csv', delimiter=',', skiprows=1),
        np.loadtxt(directory / 'state.csv', delimiter=',', skiprows=1),
        np.loadtxt(directory / 'control.csv', delimiter=',', skiprows=1),
    )
