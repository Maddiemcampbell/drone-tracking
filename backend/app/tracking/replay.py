"""Bounded, causal replay coordination for delayed sensor observations.

The Kalman filter remains a chronological same-time update primitive. This
module owns arrival ordering, checkpoints, delayed-observation acceptance, and
the rule that already emitted output timestamps are never rewritten.
"""

from copy import deepcopy
from dataclasses import dataclass
import math

from app.schemas.models import DelayedObservationRejection, SensorObservation, TrackEstimate, TrackUpdateDiagnostic
from app.tracking.kalman import ConstantVelocityKalmanTracker, TrackerConfig

EPSILON = 1e-9


@dataclass(frozen=True)
class ReplayConfig:
    history_window_seconds: float = 5.0

    def __post_init__(self) -> None:
        if self.history_window_seconds < 0:
            raise ValueError("replay history window must be non-negative")


@dataclass
class _Checkpoint:
    """Tracker state immediately before a measurement-time group."""

    measurement_timestamp: float
    tracker: ConstantVelocityKalmanTracker
    diagnostics: list[TrackUpdateDiagnostic]


class BoundedReplayTracker:
    """Replay arrived observations while preserving emitted output history."""

    def __init__(self, tracker_config: TrackerConfig | None = None, replay_config: ReplayConfig | None = None) -> None:
        self.tracker_config = tracker_config or TrackerConfig()
        self.replay_config = replay_config or ReplayConfig()
        self._accepted: list[SensorObservation] = []
        self._checkpoints: list[_Checkpoint] = [self._initial_checkpoint()]
        self._tracker = ConstantVelocityKalmanTracker(self.tracker_config)
        self._diagnostics: list[TrackUpdateDiagnostic] = []
        self._rejections: list[DelayedObservationRejection] = []

    def run(
        self,
        observations: list[SensorObservation],
        output_timestamps: list[float],
    ) -> tuple[list[TrackEstimate], list[TrackUpdateDiagnostic], list[DelayedObservationRejection]]:
        """Process arrivals and emit estimates only at requested output times."""
        arrivals = sorted(
            observations,
            key=lambda observation: (
                observation.availability_timestamp,
                observation.measurement_timestamp,
                observation.sensor_id,
            ),
        )
        outputs = sorted(set(output_timestamps))
        event_times = sorted(set(outputs + [
            observation.availability_timestamp
            for observation in arrivals
            if observation.availability_timestamp <= (outputs[-1] if outputs else -math.inf) + EPSILON
        ]))
        output_set = set(outputs)
        estimates: list[TrackEstimate] = []
        arrival_index = 0
        for current_time in event_times:
            newly_accepted: list[SensorObservation] = []
            while arrival_index < len(arrivals) and arrivals[arrival_index].availability_timestamp <= current_time + EPSILON:
                observation = arrivals[arrival_index]
                arrival_index += 1
                if observation.availability_timestamp + EPSILON < observation.measurement_timestamp:
                    self._reject(observation, "availability_before_measurement")
                    continue
                if current_time - observation.measurement_timestamp > self.replay_config.history_window_seconds + EPSILON:
                    self._reject(observation, "outside_history_window")
                    continue
                self._accepted.append(observation)
                newly_accepted.append(observation)
            if newly_accepted:
                self._replay_to(current_time, min(observation.measurement_timestamp for observation in newly_accepted))
            elif self._tracker.initialized and self._tracker.timestamp is not None and self._tracker.timestamp < current_time - EPSILON:
                self._tracker.predict_to(current_time)
            if current_time in output_set and self._tracker.initialized:
                estimates.append(self._tracker.snapshot())
        return estimates, self._diagnostics, self._rejections

    @property
    def rejected_observation_count(self) -> int:
        return len(self._rejections)

    def _reject(self, observation: SensorObservation, reason: str) -> None:
        self._rejections.append(
            DelayedObservationRejection(
                sensor_id=observation.sensor_id,
                measurement_timestamp=observation.measurement_timestamp,
                availability_timestamp=observation.availability_timestamp,
                reason=reason,
            )
        )

    def _initial_checkpoint(self) -> _Checkpoint:
        return _Checkpoint(-math.inf, ConstantVelocityKalmanTracker(self.tracker_config), [])

    def _replay_to(self, current_time: float, replay_from_timestamp: float) -> None:
        """Restore before the delayed timestamp and replay only arrived data."""
        candidates = [
            checkpoint
            for checkpoint in self._checkpoints
            if checkpoint.measurement_timestamp < replay_from_timestamp - EPSILON
        ]
        checkpoint = max(candidates, key=lambda item: item.measurement_timestamp) if candidates else self._initial_checkpoint()
        tracker = deepcopy(checkpoint.tracker)
        diagnostics = deepcopy(checkpoint.diagnostics)
        ordered = sorted(self._accepted, key=lambda observation: (observation.measurement_timestamp, observation.sensor_id))
        groups: list[list[SensorObservation]] = []
        for observation in ordered:
            if not groups or abs(groups[-1][0].measurement_timestamp - observation.measurement_timestamp) > EPSILON:
                groups.append([observation])
            else:
                groups[-1].append(observation)

        checkpoints = [checkpoint]
        for group in groups:
            group_time = group[0].measurement_timestamp
            if group_time < checkpoint.measurement_timestamp - EPSILON:
                continue
            checkpoints.append(_Checkpoint(group_time, deepcopy(tracker), deepcopy(diagnostics)))
            for observation in group:
                # Delivery has already been validated here. The core filter
                # receives a same-time replay copy and never sees latency.
                replay_observation = observation.model_copy(
                    update={"availability_timestamp": observation.measurement_timestamp}
                )
                _, diagnostic = tracker.process_observation(replay_observation)
                diagnostics.append(diagnostic)
        if tracker.initialized and tracker.timestamp is not None and tracker.timestamp < current_time - EPSILON:
            tracker.predict_to(current_time)
        self._tracker = tracker
        self._diagnostics = diagnostics
        threshold = current_time - self.replay_config.history_window_seconds
        self._checkpoints = [
            item for item in checkpoints if item.measurement_timestamp == -math.inf or item.measurement_timestamp >= threshold - EPSILON
        ]
