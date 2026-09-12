"""Avoidance corridor profile helpers independent of ROS 2 and CasADi."""

import math


def cosine_blend(start, finish, ratio):
    """Blend two values with zero slope at both endpoints."""
    blend = 0.5 * (1.0 - math.cos(math.pi * ratio))
    return start + (finish - start) * blend


def avoidance_offset(progress, stations, amplitude):
    """Evaluate the +amplitude to -amplitude opposite-side corridor."""
    s0, s1, s2, s3, s4, s5 = stations
    if s0 <= progress < s1:
        return cosine_blend(0.0, amplitude, (progress - s0) / (s1 - s0))
    if s1 <= progress <= s2:
        return amplitude
    if s2 < progress < s3:
        return cosine_blend(amplitude, -amplitude, (progress - s2) / (s3 - s2))
    if s3 <= progress <= s4:
        return -amplitude
    if s4 < progress <= s5:
        return cosine_blend(-amplitude, 0.0, (progress - s4) / (s5 - s4))
    return 0.0
