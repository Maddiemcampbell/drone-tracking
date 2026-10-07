"""A small constant-velocity Kalman filter for Cartesian position observations."""

import numpy as np

from app.schemas.models import SensorObservation, TrackEstimate


class ConstantVelocityKalmanFilter:
    """Estimate [x, y, vx, vy] from position-only observations."""

    def __init__(self, process_acceleration_std: float = 1.0) -> None:
        self.process_acceleration_std = process_acceleration_std
        self.state: np.ndarray | None = None
        self.covariance: np.ndarray | None = None
        self.timestamp: float | None = None

    def update(self, observation: SensorObservation) -> TrackEstimate:
        measurement_time = observation.measurement_timestamp
        measurement = np.asarray(observation.measurement_values, dtype=float)
        measurement_covariance = np.asarray(observation.measurement_covariance, dtype=float)

        if self.state is None:
            self.state = np.array([measurement[0], measurement[1], 0.0, 0.0])
            self.covariance = np.diag([measurement_covariance[0, 0], measurement_covariance[1, 1], 100.0, 100.0])
            self.timestamp = measurement_time
        else:
            assert self.covariance is not None and self.timestamp is not None
            dt = measurement_time - self.timestamp
            if dt < 0:
                raise ValueError("Observations must arrive in non-decreasing timestamp order")
            transition = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
            q = self.process_acceleration_std**2
            process_noise = q * np.array([[dt**4 / 4, 0, dt**3 / 2, 0], [0, dt**4 / 4, 0, dt**3 / 2], [dt**3 / 2, 0, dt**2, 0], [0, dt**3 / 2, 0, dt**2]])
            self.state = transition @ self.state
            self.covariance = transition @ self.covariance @ transition.T + process_noise
            self.timestamp = measurement_time

            measurement_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
            innovation = measurement - measurement_matrix @ self.state
            innovation_covariance = measurement_matrix @ self.covariance @ measurement_matrix.T + measurement_covariance
            gain = self.covariance @ measurement_matrix.T @ np.linalg.inv(innovation_covariance)
            self.state = self.state + gain @ innovation
            identity = np.eye(4)
            self.covariance = (identity - gain @ measurement_matrix) @ self.covariance

        return TrackEstimate(
            track_id="track-1",
            timestamp=measurement_time,
            estimated_position=self.state[:2].tolist(),
            estimated_velocity=self.state[2:].tolist(),
            state_covariance=self.covariance.tolist(),
        )


def estimate(observations: list[SensorObservation]) -> list[TrackEstimate]:
    tracker = ConstantVelocityKalmanFilter()
    return [tracker.update(observation) for observation in observations]
