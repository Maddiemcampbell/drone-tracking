"""Prediction-only constant-velocity tracking primitives for Cartesian observations.

This module deliberately stops before a measurement update. It is useful for
teaching the prediction model, but its output is not a finished track estimate.
"""

from dataclasses import dataclass

import numpy as np

from app.schemas.models import SensorObservation, TrackEstimate

STATE_SIZE = 4


@dataclass(frozen=True)
class TrackerConfig:
    """Configuration for the prediction-only tracker.

    ``acceleration_noise_spectral_density`` is q in m²/s³. The initial
    velocity uncertainty is a standard deviation in m/s.
    """

    initial_velocity_std_mps: float = 10.0
    acceleration_noise_spectral_density: float = 1.0

    def __post_init__(self) -> None:
        if self.initial_velocity_std_mps < 0:
            raise ValueError("initial velocity standard deviation must be non-negative")
        if self.acceleration_noise_spectral_density < 0:
            raise ValueError("acceleration noise spectral density must be non-negative")


def constant_velocity_transition(dt: float) -> np.ndarray:
    """Return F(dt) for state ordering [x, y, vx, vy]."""
    if dt < 0:
        raise ValueError("elapsed time must be non-negative")
    return np.array(
        [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        dtype=float,
    )


def continuous_white_acceleration_process_noise(dt: float, q: float) -> np.ndarray:
    """Return Q(dt) for continuous white acceleration noise with spectral density q."""
    if dt < 0:
        raise ValueError("elapsed time must be non-negative")
    if q < 0:
        raise ValueError("acceleration noise spectral density must be non-negative")
    dt2 = dt**2
    dt3 = dt**3
    return q * np.array(
        [[dt3 / 3, 0.0, dt2 / 2, 0.0], [0.0, dt3 / 3, 0.0, dt2 / 2], [dt2 / 2, 0.0, dt, 0.0], [0.0, dt2 / 2, 0.0, dt]],
        dtype=float,
    )


def _symmetric(matrix: np.ndarray) -> np.ndarray:
    return (matrix + matrix.T) / 2.0


class ConstantVelocityKalmanTracker:
    """Initialize from one Cartesian observation and predict without updates."""

    def __init__(self, config: TrackerConfig | None = None) -> None:
        self.config = config or TrackerConfig()
        self.state: np.ndarray | None = None
        self.covariance: np.ndarray | None = None
        self.timestamp: float | None = None

    @property
    def initialized(self) -> bool:
        return self.state is not None

    def initialize(self, observation: SensorObservation) -> TrackEstimate:
        """Initialize from the first available Cartesian observation only."""
        if observation.measurement_type != "cartesian_position":
            raise ValueError("the tracker accepts Cartesian position observations only")
        values = np.asarray(observation.measurement_values, dtype=float)
        measurement_covariance = np.asarray(observation.measurement_covariance, dtype=float)
        if values.shape != (2,):
            raise ValueError("Cartesian observations must contain x and y")
        if measurement_covariance.shape != (2, 2):
            raise ValueError("position covariance must be 2x2")
        if observation.measurement_timestamp < 0:
            raise ValueError("measurement timestamp must be non-negative")

        velocity_variance = self.config.initial_velocity_std_mps**2
        self.state = np.array([values[0], values[1], 0.0, 0.0], dtype=float)
        self.covariance = np.zeros((STATE_SIZE, STATE_SIZE), dtype=float)
        self.covariance[:2, :2] = measurement_covariance
        self.covariance[2:, 2:] = np.eye(2) * velocity_variance
        self.covariance = _symmetric(self.covariance)
        self.timestamp = observation.measurement_timestamp
        return self._estimate()

    def predict_to(self, timestamp: float) -> TrackEstimate:
        """Predict to an absolute simulation timestamp without a measurement update."""
        if not self.initialized:
            raise RuntimeError("tracker is uninitialized; initialize it with an observation first")
        assert self.state is not None and self.covariance is not None and self.timestamp is not None
        dt = timestamp - self.timestamp
        if dt < 0:
            raise ValueError("prediction timestamp must not precede the tracker timestamp")
        transition = constant_velocity_transition(dt)
        process_noise = continuous_white_acceleration_process_noise(dt, self.config.acceleration_noise_spectral_density)
        self.state = transition @ self.state
        self.covariance = _symmetric(transition @ self.covariance @ transition.T + process_noise)
        self.timestamp = timestamp
        return self._estimate()

    def _estimate(self) -> TrackEstimate:
        assert self.state is not None and self.covariance is not None and self.timestamp is not None
        return TrackEstimate(
            track_id="track-1",
            timestamp=self.timestamp,
            estimated_position=self.state[:2].tolist(),
            estimated_velocity=self.state[2:].tolist(),
            state_covariance=self.covariance.tolist(),
        )


def estimate(observations: list[SensorObservation], config: TrackerConfig | None = None) -> list[TrackEstimate]:
    """Return initialization plus prediction-only outputs at observation times.

    Observations provide timestamps for the prediction schedule, but are never
    used to correct the predicted state after initialization.
    """
    if not observations:
        return []
    tracker = ConstantVelocityKalmanTracker(config)
    estimates = [tracker.initialize(observations[0])]
    for observation in observations[1:]:
        if observation.measurement_type != "cartesian_position":
            raise ValueError("the tracker accepts Cartesian position observations only")
        estimates.append(tracker.predict_to(observation.measurement_timestamp))
    return estimates
