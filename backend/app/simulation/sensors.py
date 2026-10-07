import numpy as np
from app.schemas.models import SensorObservation, TargetState

def observe(state: TargetState, noise_std: float, rng: np.random.Generator) -> SensorObservation:
    variance = noise_std**2
    values = np.array([state.x, state.y]) + rng.normal(0, noise_std, size=2)
    return SensorObservation(sensor_id="position-sensor-1", measurement_timestamp=state.timestamp, availability_timestamp=state.timestamp, measurement_type="cartesian_position", measurement_values=values.tolist(), measurement_covariance=[[variance, 0.0], [0.0, variance]])
