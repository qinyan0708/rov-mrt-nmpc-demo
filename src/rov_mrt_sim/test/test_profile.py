"""Tests for the prescribed opposite-side avoidance corridor."""

import math
import unittest

from rov_mrt_sim.profile import avoidance_offset


class ProfileTest(unittest.TestCase):
    """Check offset values at every corridor phase."""

    def test_profile_key_stations(self):
        stations = [1.0, 2.0, 3.0, 5.0, 6.0, 7.0]
        self.assertEqual(avoidance_offset(0.0, stations, 3.5), 0.0)
        self.assertTrue(math.isclose(avoidance_offset(2.0, stations, 3.5), 3.5))
        self.assertTrue(
            math.isclose(avoidance_offset(4.0, stations, 3.5), 0.0, abs_tol=1e-12)
        )
        self.assertTrue(math.isclose(avoidance_offset(5.5, stations, 3.5), -3.5))
        self.assertEqual(avoidance_offset(8.0, stations, 3.5), 0.0)


if __name__ == '__main__':
    unittest.main()
