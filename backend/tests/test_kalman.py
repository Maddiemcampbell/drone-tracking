import numpy as np
import pytest

from app.schemas.models import SensorObservation
from app.tracking.kalman import (
    ConstantVelocityKalmanTracker,
    TrackerConfig,
    constant_velocity_transition,
    continuous_white_acceleration_process_noise,
    estimate,
)


def observation(timestamp: float, x: float, y: float, measurement_type: str = "cartesian_position", covariance: list[list[float]] | None = None) -> SensorObservation:
    return SensorObservation(
        sensor_id="sensor-1",
        measurement_timestamp=timestamp,
        availability_timestamp=timestamp,
        measurement_type=measurement_type,
        measurement_values=[x, y],
        measurement_covariance=covariance or [[0.01, 0], [0, 0.01]],
    )


def test_tracker_is_uninitialized_until_first_cartesian_observation():
    tracker = ConstantVelocityKalmanTracker(TrackerConfig(initial_velocity_std_mps=3))
    assert tracker.initialized is False
    with pytest.raises(RuntimeError, match="uninitialized"):
        tracker.predict_to(0)

    initial = tracker.initialize(observation(2, 12, -3, covariance=[[4, 0.5], [0.5, 9]]))
    covariance = np.asarray(initial.state_covariance)
    assert tracker.initialized is True
    assert initial.timestamp == 2
    assert initial.estimated_position == pytest.approx([12, -3])
    assert initial.estimated_velocity == pytest.approx([0, 0])
    np.testing.assert_allclose(covariance, [[4, .5, 0, 0], [.5, 9, 0, 0], [0, 0, 9, 0], [0, 0, 0, 9]])


def test_constant_velocity_transition_matches_known_state():
    state = np.array([2.0, -1.0, 4.0, -3.0])
    predicted = constant_velocity_transition(2.5) @ state
    assert predicted == pytest.approx([12, -8.5, 4, -3])


def test_tracker_prediction_matches_known_state_and_variable_dt():
    tracker = ConstantVelocityKalmanTracker(TrackerConfig(acceleration_noise_spectral_density=0))
    tracker.initialize(observation(0, 0, 0))
    tracker.state = np.array([1.0, 2.0, 3.0, -4.0])
    first = tracker.predict_to(.25)
    second = tracker.predict_to(1.75)
    assert first.estimated_position == pytest.approx([1.75, 1.0])
    assert second.estimated_position == pytest.approx([6.25, -5.0])
    assert second.estimated_velocity == pytest.approx([3, -4])


def test_process_noise_matches_continuous_white_acceleration_model():
    q = 2.5
    dt = 2.0
    expected = q * np.array([[dt**3 / 3, 0, dt**2 / 2, 0], [0, dt**3 / 3, 0, dt**2 / 2], [dt**2 / 2, 0, dt, 0], [0, dt**2 / 2, 0, dt]])
    assert continuous_white_acceleration_process_noise(dt, q) == pytest.approx(expected)


def test_prediction_covariance_is_symmetric_psd_and_grows_with_process_noise():
    tracker = ConstantVelocityKalmanTracker(TrackerConfig(initial_velocity_std_mps=1, acceleration_noise_spectral_density=3))
    tracker.initialize(observation(0, 0, 0))
    initial_trace = np.trace(tracker.covariance)
    predicted = tracker.predict_to(4)
    covariance = np.asarray(predicted.state_covariance)
    assert covariance == pytest.approx(covariance.T)
    assert np.min(np.linalg.eigvalsh(covariance)) >= -1e-10
    assert np.trace(covariance) > initial_trace


def test_prediction_rejects_negative_elapsed_time():
    tracker = ConstantVelocityKalmanTracker()
    tracker.initialize(observation(2, 1, 1))
    with pytest.raises(ValueError, match="precede"):
        tracker.predict_to(1)
    with pytest.raises(ValueError, match="non-negative"):
        constant_velocity_transition(-1)
    with pytest.raises(ValueError, match="non-negative"):
        continuous_white_acceleration_process_noise(-1, 1)


def test_prediction_only_does_not_apply_measurement_updates():
    estimates = estimate([observation(0, 0, 0), observation(1, 100, 100), observation(2, -50, 40)], TrackerConfig(acceleration_noise_spectral_density=0))
    assert estimates[1].estimated_position == pytest.approx([0, 0])
    assert estimates[2].estimated_position == pytest.approx([0, 0])
    assert estimates[2].estimated_velocity == pytest.approx([0, 0])


def test_tracker_accepts_cartesian_observations_only():
    radar = observation(0, 10, .5, measurement_type="range_bearing")
    with pytest.raises(ValueError, match="Cartesian"):
        ConstantVelocityKalmanTracker().initialize(radar)
    with pytest.raises(ValueError, match="Cartesian"):
        estimate([radar])


def test_tracker_configuration_rejects_negative_uncertainty_or_spectral_density():
    with pytest.raises(ValueError):
        TrackerConfig(initial_velocity_std_mps=-1)
    with pytest.raises(ValueError):
        TrackerConfig(acceleration_noise_spectral_density=-1)
