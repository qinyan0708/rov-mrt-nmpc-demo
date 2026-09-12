"""Pure-Python kinematics shared by the simulator and unit tests."""

import math

import numpy as np


def wrap_to_pi(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def derivative(state, control, length):
    """Return the five-state articulated-vehicle kinematic derivative."""
    _, _, heading, speed, steering_angle = state
    acceleration, steering_rate = control
    return np.asarray([
        speed * math.cos(heading),
        speed * math.sin(heading),
        speed / length * math.tan(steering_angle),
        acceleration,
        steering_rate,
    ], dtype=float)


def rk4_step(state, control, sample_time, length):
    """Advance the kinematic model by one fixed RK4 step."""
    state = np.asarray(state, dtype=float)
    control = np.asarray(control, dtype=float)
    k1 = derivative(state, control, length)
    k2 = derivative(state + 0.5 * sample_time * k1, control, length)
    k3 = derivative(state + 0.5 * sample_time * k2, control, length)
    k4 = derivative(state + sample_time * k3, control, length)
    result = state + sample_time * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    result[2] = wrap_to_pi(result[2])
    result[4] = wrap_to_pi(result[4])
    return result


def collision_centers(state, length, rov_offset, rho_mrt, rho_rov):
    """Return MRT and ROV collision-circle centers for a vehicle state."""
    state = np.asarray(state, dtype=float)
    direction_mrt = np.asarray([math.cos(state[2]), math.sin(state[2])])
    hinge = state[:2] + length * direction_mrt
    rov_heading = state[2] + state[4]
    direction_rov = np.asarray([math.cos(rov_heading), math.sin(rov_heading)])
    rov_center = hinge + rov_offset * direction_rov
    mrt = np.asarray([state[:2] + rho * direction_mrt for rho in rho_mrt])
    rov = np.asarray([rov_center + rho * direction_rov for rho in rho_rov])
    return mrt, rov


def braking_control(speed, steering_angle, deceleration, gain, max_rate):
    """Return bounded braking and steering-centering commands."""
    acceleration = -abs(deceleration) if speed > 1.0e-6 else 0.0
    steering_rate = float(np.clip(-gain * steering_angle, -max_rate, max_rate))
    return np.asarray([acceleration, steering_rate])
