import math
from typing import Literal

import numpy as np
from app.schemas.models import SensorObservation, TargetState


def normalize_angle_radians(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def observe(
    state: TargetState,
    noise_std: float | None = None,
    rng: np.random.Generator | None = None,
    *,
    noise_std_x: float | None = None,
    noise_std_y: float | None = None,
    bias_x: float = 0.0,
    bias_y: float = 0.0,
    sensor_id: str = "position-sensor-1",
    sensor_position: tuple[float, float] = (0.0, 0.0),
) -> SensorObservation | None:
    """Create one world-frame Cartesian observation with independent Gaussian noise.

    ``noise_std`` is retained as a compatibility shorthand for equal x/y noise.
    """
    if rng is None:
        raise ValueError("rng is required")
    if noise_std is not None:
        noise_std_x = noise_std if noise_std_x is None else noise_std_x
        noise_std_y = noise_std if noise_std_y is None else noise_std_y
    if noise_std_x is None or noise_std_y is None:
        raise ValueError("both x and y noise standard deviations are required")
    variance_x = noise_std_x**2
    variance_y = noise_std_y**2
    values = np.array([state.x + bias_x, state.y + bias_y]) + rng.normal(0, [noise_std_x, noise_std_y], size=2)
    return SensorObservation(
        sensor_id=sensor_id,
        measurement_timestamp=state.timestamp,
        availability_timestamp=state.timestamp,
        measurement_type="cartesian_position",
        measurement_values=values.tolist(),
        measurement_covariance=[[variance_x, 0.0], [0.0, variance_y]],
        sensor_position=list(sensor_position),
    )


def observe_range_bearing(
    state: TargetState,
    rng: np.random.Generator,
    *,
    sensor_id: str,
    sensor_position: tuple[float, float],
    sensor_heading_degrees: float,
    range_noise_std_meters: float,
    bearing_noise_std_degrees: float,
) -> SensorObservation | None:
    """Create one simplified range/bearing observation, or omit invalid samples."""
    dx = state.x - sensor_position[0]
    dy = state.y - sensor_position[1]
    true_range = math.hypot(dx, dy)
    if true_range == 0:
        return None

    sensor_heading_radians = math.radians(sensor_heading_degrees)
    true_bearing = normalize_angle_radians(math.atan2(dy, dx) - sensor_heading_radians)
    measured_range = true_range + float(rng.normal(0, range_noise_std_meters))
    if measured_range < 0:
        return None
    measured_bearing = normalize_angle_radians(true_bearing + math.radians(float(rng.normal(0, bearing_noise_std_degrees))))
    bearing_noise_std_radians = math.radians(bearing_noise_std_degrees)
    return SensorObservation(
        sensor_id=sensor_id,
        measurement_timestamp=state.timestamp,
        availability_timestamp=state.timestamp,
        measurement_type="range_bearing",
        measurement_values=[measured_range, measured_bearing],
        measurement_covariance=[[range_noise_std_meters**2, 0.0], [0.0, bearing_noise_std_radians**2]],
        sensor_position=list(sensor_position),
    )


def observe_sensor(
    state: TargetState,
    rng: np.random.Generator,
    *,
    sensor_type: Literal["cartesian_position", "range_bearing"],
    sensor_id: str,
    sensor_position: tuple[float, float],
    noise_std_x: float,
    noise_std_y: float,
    bias_x: float,
    bias_y: float,
    sensor_heading_degrees: float,
    range_noise_std_meters: float,
    bearing_noise_std_degrees: float,
) -> SensorObservation | None:
    if sensor_type == "range_bearing":
        return observe_range_bearing(
            state,
            rng,
            sensor_id=sensor_id,
            sensor_position=sensor_position,
            sensor_heading_degrees=sensor_heading_degrees,
            range_noise_std_meters=range_noise_std_meters,
            bearing_noise_std_degrees=bearing_noise_std_degrees,
        )
    return observe(
        state,
        rng=rng,
        noise_std_x=noise_std_x,
        noise_std_y=noise_std_y,
        bias_x=bias_x,
        bias_y=bias_y,
        sensor_id=sensor_id,
        sensor_position=sensor_position,
    )
