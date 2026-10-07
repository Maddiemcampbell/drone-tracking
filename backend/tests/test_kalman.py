import numpy as np
import pytest

from app.schemas.models import SensorObservation
from app.tracking.kalman import (
    ConstantVelocityKalmanTracker,
    TrackerConfig,
    constant_velocity_transition,
    continuous_white_acceleration_process_noise,
    estimate,
    estimate_at_timestamps,
    estimate_at_timestamps_with_diagnostics,
)


def observation(timestamp: float, x: float, y: float, measurement_type: str = "cartesian_position", covariance: list[list[float]] | None = None, sensor_id: str = "sensor-1") -> SensorObservation:
    return SensorObservation(
        sensor_id=sensor_id,
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


def test_measurement_update_moves_estimate_toward_observation():
    tracker = ConstantVelocityKalmanTracker(TrackerConfig(acceleration_noise_spectral_density=0))
    tracker.initialize(observation(0, 0, 0, covariance=[[4, 0], [0, 4]]))
    updated = tracker.update(observation(0, 10, 8, covariance=[[1, 0], [0, 1]]))
    assert 0 < updated.estimated_position[0] < 10
    assert 0 < updated.estimated_position[1] < 8
    assert updated.measurement_updated is True
    assert updated.last_measurement_timestamp == 0


def test_simultaneous_sensor_updates_follow_sensor_id_order_and_expose_diagnostics():
    config = TrackerConfig(acceleration_noise_spectral_density=0)
    first = observation(0, 0, 0, sensor_id="sensor-b")
    second = observation(0, 10, 4, sensor_id="sensor-a")
    estimates, diagnostics = estimate_at_timestamps_with_diagnostics([first, second], [0], config)
    assert [diagnostic.sensor_id for diagnostic in diagnostics] == ["sensor-a", "sensor-b"]
    assert diagnostics[0].initialized is True and diagnostics[0].updated is False
    assert diagnostics[1].initialized is False and diagnostics[1].updated is True
    assert len(estimates) == 1


def test_sequential_simultaneous_updates_match_joint_linear_update():
    state = np.array([1.0, -2.0, .5, -1.0])
    covariance = np.diag([10.0, 12.0, 2.0, 3.0])
    first = observation(0, 2, 4, covariance=[[4, 0], [0, 5]], sensor_id="sensor-a")
    second = observation(0, 5, -1, covariance=[[1, 0], [0, 2]], sensor_id="sensor-b")
    tracker = ConstantVelocityKalmanTracker(TrackerConfig(acceleration_noise_spectral_density=0))
    tracker.state = state.copy()
    tracker.covariance = covariance.copy()
    tracker.timestamp = 0
    tracker.last_measurement_timestamp = 0
    tracker.process_observation(first)
    tracker.process_observation(second)

    measurement_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
    joint_matrix = np.vstack([measurement_matrix, measurement_matrix])
    joint_measurement = np.array(first.measurement_values + second.measurement_values, dtype=float)
    joint_covariance = np.diag([4.0, 5.0, 1.0, 2.0])
    innovation = joint_measurement - joint_matrix @ state
    innovation_covariance = joint_matrix @ covariance @ joint_matrix.T + joint_covariance
    gain = covariance @ joint_matrix.T @ np.linalg.inv(innovation_covariance)
    expected_state = state + gain @ innovation
    identity = np.eye(4)
    expected_covariance = (identity - gain @ joint_matrix) @ covariance @ (identity - gain @ joint_matrix).T + gain @ joint_covariance @ gain.T
    np.testing.assert_allclose(tracker.state, expected_state, atol=1e-10)
    np.testing.assert_allclose(tracker.covariance, expected_covariance, atol=1e-10)


def test_noisier_measurement_has_less_influence():
    def update_with_noise(noise: float) -> float:
        tracker = ConstantVelocityKalmanTracker(TrackerConfig(acceleration_noise_spectral_density=0))
        tracker.state = np.array([0.0, 0.0, 0.0, 0.0])
        tracker.covariance = np.diag([1.0, 1.0, 1.0, 1.0])
        tracker.timestamp = 0
        tracker.last_measurement_timestamp = 0
        return tracker.process_observation(observation(0, 10, 0, covariance=[[noise, 0], [0, noise]]) )[0].estimated_position[0]

    precise = update_with_noise(1)
    noisy = update_with_noise(100)
    assert 0 < noisy < precise < 10


def test_informative_measurement_reduces_position_uncertainty():
    tracker = ConstantVelocityKalmanTracker(TrackerConfig(acceleration_noise_spectral_density=0))
    tracker.initialize(observation(0, 0, 0, covariance=[[4, 0], [0, 9]]))
    before = np.diag(tracker.covariance)[:2].copy()
    updated = tracker.update(observation(0, 1, 1, covariance=[[.25, 0], [0, .25]]))
    after = np.diag(np.asarray(updated.state_covariance))[:2]
    assert np.all(after < before)


def test_zero_noise_stationary_update_handles_singular_innovation_covariance():
    tracker = ConstantVelocityKalmanTracker(TrackerConfig(initial_velocity_std_mps=0, acceleration_noise_spectral_density=0))
    tracker.initialize(observation(0, 4, -2, covariance=[[0, 0], [0, 0]]))
    updated = tracker.update(observation(0, 4, -2, covariance=[[0, 0], [0, 0]]))
    assert updated.estimated_position == pytest.approx([4, -2])
    assert np.asarray(updated.state_covariance) == pytest.approx(np.zeros((4, 4)))


def test_tracker_accepts_cartesian_observations_only():
    radar = observation(0, 10, .5, measurement_type="range_bearing")
    with pytest.raises(ValueError, match="Cartesian"):
        ConstantVelocityKalmanTracker().initialize(radar)
    with pytest.raises(ValueError, match="Cartesian"):
        estimate([radar])


def test_delayed_observations_are_rejected_for_tracking():
    delayed = observation(2, 2, 2)
    delayed.availability_timestamp = 3
    with pytest.raises(ValueError, match="delayed"):
        ConstantVelocityKalmanTracker().update(delayed)


def test_future_observations_cannot_change_past_outputs():
    config = TrackerConfig(acceleration_noise_spectral_density=0)
    first = estimate_at_timestamps([observation(0, 0, 0), observation(1, 10, 0)], [0, 1, 2], config)
    changed_future = estimate_at_timestamps([observation(0, 0, 0), observation(1, 10, 0), observation(2, 1000, 0)], [0, 1, 2], config)
    assert len(first) == len(changed_future) == 3
    assert first[0].estimated_position == pytest.approx(changed_future[0].estimated_position)
    assert first[1].estimated_position == pytest.approx(changed_future[1].estimated_position)


def test_tracker_configuration_rejects_negative_uncertainty_or_spectral_density():
    with pytest.raises(ValueError):
        TrackerConfig(initial_velocity_std_mps=-1)
    with pytest.raises(ValueError):
        TrackerConfig(acceleration_noise_spectral_density=-1)
