import numpy as np
from app.schemas.models import SensorObservation, TargetState

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
) -> SensorObservation:
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
