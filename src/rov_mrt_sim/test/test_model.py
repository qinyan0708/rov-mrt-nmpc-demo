"""Tests for the ROS-independent vehicle model."""

import math
import unittest

import numpy as np

from rov_mrt_sim.model import (
    braking_control,
    collision_centers,
    rk4_step,
    wrap_to_pi,
)


class ModelTest(unittest.TestCase):
    """Verify integration, wrapping, and articulated geometry."""

    def test_straight_constant_speed_step(self):
        state = np.asarray([0.0, 0.0, 0.0, 0.8, 0.0])
        result = rk4_step(state, [0.0, 0.0], 0.30, 3.0)
        np.testing.assert_allclose(result, [0.24, 0.0, 0.0, 0.8, 0.0])

    def test_angle_wrapping(self):
        self.assertTrue(math.isclose(wrap_to_pi(3.0 * math.pi), -math.pi))

    def test_collision_centers_at_zero_heading(self):
        mrt, rov = collision_centers(
            [0.0, 0.0, 0.0, 0.5, 0.0],
            3.0,
            1.2,
            [-0.9, 0.55, 2.0],
            [-0.45, 0.45],
        )
        np.testing.assert_allclose(mrt[:, 0], [-0.9, 0.55, 2.0])
        np.testing.assert_allclose(rov[:, 0], [3.75, 4.65])
        np.testing.assert_allclose(mrt[:, 1], 0.0)
        np.testing.assert_allclose(rov[:, 1], 0.0)

    def test_braking_control_decelerates_and_centers(self):
        control = braking_control(0.8, 0.3, 0.6, 2.0, 0.4)
        np.testing.assert_allclose(control, [-0.6, -0.4])
        stopped = braking_control(0.0, 0.0, 0.6, 2.0, 0.4)
        np.testing.assert_allclose(stopped, [0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
