"""Radar-native measurement geometry for a Cartesian target state.

This module only describes the nonlinear radar measurement model. It does not
run a tracker or access simulation truth. A later EKF or UKF can use these
functions with the tracker's predicted state.
"""

from dataclasses import dataclass
import math

import numpy as np

from app.geometry import normalize_angle_radians

STATE_SIZE = 4
MEASUREMENT_SIZE = 2
ORIGIN_EPSILON_METERS = 1e-12


def _state_vector(state: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    values = np.asarray(state, dtype=float)
    if values.shape != (STATE_SIZE,):
        raise ValueError("radar measurement state must be [x, y, vx, vy]")
    return values


@dataclass(frozen=True)
class RadarMeasurementModel:
    """Nonlinear range/bearing model for a Cartesian state.

    The state uses world-frame meters and meters/second. The measurement is
    ``[range_meters, bearing_radians]`` in the sensor frame. Sensor heading is
    accepted in degrees at this boundary and converted to radians internally.
    """

    sensor_position: tuple[float, float] = (0.0, 0.0)
    sensor_heading_degrees: float = 0.0

    def __post_init__(self) -> None:
        if len(self.sensor_position) != 2:
            raise ValueError("radar sensor position must contain x and y")
        if not all(math.isfinite(value) for value in self.sensor_position):
            raise ValueError("radar sensor position must be finite")
        if not math.isfinite(self.sensor_heading_degrees):
            raise ValueError("radar sensor heading must be finite")

    @property
    def sensor_heading_radians(self) -> float:
        return math.radians(self.sensor_heading_degrees)

    def predict(self, state: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        """Convert a Cartesian state to the expected radar measurement."""
        values = _state_vector(state)
        dx = values[0] - self.sensor_position[0]
        dy = values[1] - self.sensor_position[1]
        range_meters = math.hypot(dx, dy)
        if range_meters <= ORIGIN_EPSILON_METERS:
            raise ValueError("radar bearing is undefined at the sensor origin")
        bearing_radians = normalize_angle_radians(
            math.atan2(dy, dx) - self.sensor_heading_radians
        )
        return np.array([range_meters, bearing_radians], dtype=float)

    def jacobian(self, state: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        """Return dh/dx for range and bearing relative to [x, y, vx, vy]."""
        values = _state_vector(state)
        dx = values[0] - self.sensor_position[0]
        dy = values[1] - self.sensor_position[1]
        range_meters = math.hypot(dx, dy)
        if range_meters <= ORIGIN_EPSILON_METERS:
            raise ValueError("radar bearing Jacobian is undefined at the sensor origin")
        range_squared = range_meters**2
        return np.array(
            [
                [dx / range_meters, dy / range_meters, 0.0, 0.0],
                [-dy / range_squared, dx / range_squared, 0.0, 0.0],
            ],
            dtype=float,
        )

    @staticmethod
    def covariance(
        range_noise_std_meters: float,
        bearing_noise_std_degrees: float,
    ) -> np.ndarray:
        """Return diag(sigma_range², sigma_bearing_radians²).

        The diagonal deliberately has mixed units: meters squared for range
        and radians squared for bearing. Bias is not part of this covariance.
        """
        if range_noise_std_meters < 0:
            raise ValueError("range noise standard deviation must be non-negative")
        if bearing_noise_std_degrees < 0:
            raise ValueError("bearing noise standard deviation must be non-negative")
        bearing_noise_std_radians = math.radians(bearing_noise_std_degrees)
        return np.diag(
            [range_noise_std_meters**2, bearing_noise_std_radians**2]
        ).astype(float)


__all__ = ["MEASUREMENT_SIZE", "ORIGIN_EPSILON_METERS", "RadarMeasurementModel"]
