"""Small shared geometry helpers."""

import math


def normalize_angle_radians(angle: float) -> float:
    """Normalize an angle to the half-open interval [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
