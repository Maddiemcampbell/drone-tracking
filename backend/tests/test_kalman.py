import pytest

from app.schemas.models import SensorObservation
from app.tracking.kalman import ConstantVelocityKalmanFilter, estimate


def observation(timestamp: float, x: float, y: float) -> SensorObservation:
    return SensorObservation(
        sensor_id="sensor-1",
        measurement_timestamp=timestamp,
        availability_timestamp=timestamp,
        measurement_type="cartesian_position",
        measurement_values=[x, y],
        measurement_covariance=[[0.01, 0], [0, 0.01]],
    )


def test_tracker_uses_observations_only_and_estimates_velocity():
    estimates = estimate([observation(0, 0, 0), observation(1, 10, 5), observation(2, 20, 10)])
    assert estimates[-1].estimated_position == pytest.approx([20, 10], abs=0.2)
    assert estimates[-1].estimated_velocity == pytest.approx([10, 5], abs=0.5)


def test_tracker_rejects_out_of_order_observations():
    tracker = ConstantVelocityKalmanFilter()
    tracker.update(observation(1, 1, 1))
    with pytest.raises(ValueError, match="non-decreasing"):
        tracker.update(observation(0, 0, 0))
