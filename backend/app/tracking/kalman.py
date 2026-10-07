"""Single-target Cartesian constant-velocity Kalman tracking primitives.

The tracker is deliberately independent of simulation truth and motion-event
configuration. It consumes observations, timestamps, and tracker settings.
"""

from dataclasses import dataclass

import numpy as np

from app.schemas.models import SensorObservation, TrackEstimate, TrackUpdateDiagnostic

STATE_SIZE = 4
MEASUREMENT_SIZE = 2
MEASUREMENT_MATRIX = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])


@dataclass(frozen=True)
class TrackerConfig:
    """Configuration for the Cartesian constant-velocity tracker."""

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


def _solve_innovation_covariance(innovation_covariance: np.ndarray, right_hand_side: np.ndarray) -> np.ndarray:
    """Solve S X = B, using least squares only for a singular valid S.

    With zero measurement noise and zero predicted position uncertainty, S can
    be exactly singular. Least squares preserves the valid zero-noise model;
    no undocumented diagonal noise is added.
    """
    try:
        return np.linalg.solve(innovation_covariance, right_hand_side)
    except np.linalg.LinAlgError:
        solution, _, _, _ = np.linalg.lstsq(innovation_covariance, right_hand_side, rcond=None)
        if not np.allclose(innovation_covariance @ solution, right_hand_side, atol=1e-9):
            raise ValueError("innovation covariance is singular and inconsistent") from None
        return solution


class ConstantVelocityKalmanTracker:
    """Causal Cartesian tracker with initialization, prediction, and updates."""

    def __init__(self, config: TrackerConfig | None = None) -> None:
        self.config = config or TrackerConfig()
        self.state: np.ndarray | None = None
        self.covariance: np.ndarray | None = None
        self.timestamp: float | None = None
        self.last_measurement_timestamp: float | None = None
        self.measurement_updated: bool = False

    @property
    def initialized(self) -> bool:
        return self.state is not None

    def initialize(self, observation: SensorObservation) -> TrackEstimate:
        """Initialize position and covariance from the first Cartesian observation."""
        self._validate_cartesian_observation(observation)
        if observation.availability_timestamp != observation.measurement_timestamp:
            raise ValueError("delayed observations are not supported by the tracker")
        if observation.measurement_timestamp < 0:
            raise ValueError("measurement timestamp must be non-negative")
        values = np.asarray(observation.measurement_values, dtype=float)
        measurement_covariance = self._measurement_covariance(observation)
        velocity_variance = self.config.initial_velocity_std_mps**2
        self.state = np.array([values[0], values[1], 0.0, 0.0], dtype=float)
        self.covariance = np.zeros((STATE_SIZE, STATE_SIZE), dtype=float)
        self.covariance[:2, :2] = measurement_covariance
        self.covariance[2:, 2:] = np.eye(2) * velocity_variance
        self.covariance = _symmetric(self.covariance)
        self.timestamp = observation.measurement_timestamp
        self.last_measurement_timestamp = observation.measurement_timestamp
        self.measurement_updated = True
        return self.snapshot()

    def predict_to(self, timestamp: float) -> TrackEstimate:
        """Predict to an absolute timestamp without applying a measurement."""
        self._require_initialized()
        assert self.state is not None and self.covariance is not None and self.timestamp is not None
        dt = timestamp - self.timestamp
        if dt < 0:
            raise ValueError("prediction timestamp must not precede the tracker timestamp")
        transition = constant_velocity_transition(dt)
        process_noise = continuous_white_acceleration_process_noise(dt, self.config.acceleration_noise_spectral_density)
        self.state = transition @ self.state
        self.covariance = _symmetric(transition @ self.covariance @ transition.T + process_noise)
        self.timestamp = timestamp
        self.measurement_updated = False
        return self.snapshot()

    def update(self, observation: SensorObservation) -> TrackEstimate:
        """Predict to and incorporate one same-time Cartesian observation."""
        estimate, _ = self.process_observation(observation)
        return estimate

    def process_observation(self, observation: SensorObservation) -> tuple[TrackEstimate, TrackUpdateDiagnostic]:
        """Process one observation and expose its causal innovation diagnostic."""
        self._validate_cartesian_observation(observation)
        if observation.availability_timestamp != observation.measurement_timestamp:
            raise ValueError("delayed observations are not supported by the tracker")
        if not self.initialized:
            estimate = self.initialize(observation)
            return estimate, TrackUpdateDiagnostic(
                sensor_id=observation.sensor_id,
                measurement_timestamp=observation.measurement_timestamp,
                innovation=[0.0, 0.0],
                initialized=True,
                updated=False,
            )
        assert self.timestamp is not None and self.state is not None and self.covariance is not None
        self.predict_to(observation.measurement_timestamp)
        measurement = np.asarray(observation.measurement_values, dtype=float)
        measurement_covariance = self._measurement_covariance(observation)
        innovation = measurement - MEASUREMENT_MATRIX @ self.state
        innovation_covariance = _symmetric(MEASUREMENT_MATRIX @ self.covariance @ MEASUREMENT_MATRIX.T + measurement_covariance)
        gain = _solve_innovation_covariance(innovation_covariance, (self.covariance @ MEASUREMENT_MATRIX.T).T).T
        self.state = self.state + gain @ innovation
        identity = np.eye(STATE_SIZE)
        residual_projection = identity - gain @ MEASUREMENT_MATRIX
        self.covariance = _symmetric(residual_projection @ self.covariance @ residual_projection.T + gain @ measurement_covariance @ gain.T)
        self.timestamp = observation.measurement_timestamp
        self.last_measurement_timestamp = observation.measurement_timestamp
        self.measurement_updated = True
        return self.snapshot(), TrackUpdateDiagnostic(
            sensor_id=observation.sensor_id,
            measurement_timestamp=observation.measurement_timestamp,
            innovation=innovation.tolist(),
            initialized=False,
            updated=True,
        )

    def snapshot(self) -> TrackEstimate:
        self._require_initialized()
        assert self.state is not None and self.covariance is not None and self.timestamp is not None
        return TrackEstimate(
            track_id="track-1",
            timestamp=self.timestamp,
            estimated_position=self.state[:2].tolist(),
            estimated_velocity=self.state[2:].tolist(),
            state_covariance=self.covariance.tolist(),
            last_measurement_timestamp=self.last_measurement_timestamp,
            measurement_age_seconds=(self.timestamp - self.last_measurement_timestamp if self.last_measurement_timestamp is not None else None),
            measurement_updated=self.measurement_updated,
        )

    def _require_initialized(self) -> None:
        if not self.initialized:
            raise RuntimeError("tracker is uninitialized; initialize it with an observation first")

    @staticmethod
    def _validate_cartesian_observation(observation: SensorObservation) -> None:
        if observation.measurement_type != "cartesian_position":
            raise ValueError("the tracker accepts Cartesian position observations only")
        if len(observation.measurement_values) != MEASUREMENT_SIZE:
            raise ValueError("Cartesian observations must contain x and y")

    @staticmethod
    def _measurement_covariance(observation: SensorObservation) -> np.ndarray:
        covariance = np.asarray(observation.measurement_covariance, dtype=float)
        if covariance.shape != (MEASUREMENT_SIZE, MEASUREMENT_SIZE):
            raise ValueError("position covariance must be 2x2")
        covariance = _symmetric(covariance)
        if np.min(np.linalg.eigvalsh(covariance)) < -1e-10:
            raise ValueError("position covariance must be positive semidefinite")
        return covariance


def estimate(observations: list[SensorObservation], config: TrackerConfig | None = None) -> list[TrackEstimate]:
    """Process observations causally, returning one output per observation event."""
    tracker = ConstantVelocityKalmanTracker(config)
    estimates: list[TrackEstimate] = []
    for observation in sorted(observations, key=lambda item: (item.measurement_timestamp, item.sensor_id)):
        estimates.append(tracker.process_observation(observation)[0])
    return estimates


def estimate_at_timestamps(observations: list[SensorObservation], output_timestamps: list[float], config: TrackerConfig | None = None) -> list[TrackEstimate]:
    """Return causal estimates while preserving the established API."""
    estimates, _ = estimate_at_timestamps_with_diagnostics(observations, output_timestamps, config)
    return estimates


def estimate_at_timestamps_with_diagnostics(
    observations: list[SensorObservation],
    output_timestamps: list[float],
    config: TrackerConfig | None = None,
) -> tuple[list[TrackEstimate], list[TrackUpdateDiagnostic]]:
    """Merge measurement/output events and emit causal estimates at output times.

    Measurement events sort before output events at the same timestamp, so a
    measurement is incorporated before that timestamp's estimate is emitted.
    Outputs before the first available observation are omitted because the
    tracker is uninitialized. Measurements at the same time are ordered by
    sensor ID and are processed sequentially without an extra prediction step.
    """
    for observation in observations:
        if observation.availability_timestamp != observation.measurement_timestamp:
            raise ValueError("delayed observations are not supported by the tracker")
    tracker = ConstantVelocityKalmanTracker(config)
    events = [(observation.measurement_timestamp, 0, observation.sensor_id, "measurement", observation) for observation in observations]
    events.extend((timestamp, 1, "", "output", None) for timestamp in output_timestamps)
    events.sort(key=lambda event: (event[0], event[1], event[2]))
    estimates: list[TrackEstimate] = []
    diagnostics: list[TrackUpdateDiagnostic] = []
    for timestamp, _, _, event_type, observation in events:
        if event_type == "measurement":
            assert observation is not None
            _, diagnostic = tracker.process_observation(observation)
            diagnostics.append(diagnostic)
        elif tracker.initialized:
            if tracker.timestamp is not None and timestamp > tracker.timestamp:
                tracker.predict_to(timestamp)
            estimates.append(tracker.snapshot())
    return estimates, diagnostics
