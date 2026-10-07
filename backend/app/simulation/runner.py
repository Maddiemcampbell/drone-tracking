import math
from dataclasses import dataclass
from typing import Literal

import numpy as np

from app.schemas.models import SensorOutage, SimulationConfig, SimulationResult
from app.simulation.motion import advance_with_turns
from app.simulation.sensors import observe_sensor
from app.simulation.world import initial_state
from app.tracking.kalman import TrackerConfig, estimate_at_timestamps

EPSILON = 1e-9


@dataclass(frozen=True)
class _ScheduledSensor:
    sensor_id: str
    sensor_type: Literal["cartesian_position", "range_bearing"]
    enabled: bool
    measurement_interval_seconds: float
    sampling_start_offset_seconds: float
    outage_windows: list[SensorOutage]
    sensor_position: tuple[float, float]
    noise_std_x: float
    noise_std_y: float
    bias_x: float
    bias_y: float
    sensor_heading_degrees: float
    range_noise_std_meters: float
    bearing_noise_std_degrees: float


def _is_sensor_outage(timestamp: float, outages: list[SensorOutage]) -> bool:
    return any(window.start_time_seconds <= timestamp < window.end_time_seconds for window in outages)


def _schedule_times(sensor: _ScheduledSensor, duration_seconds: float) -> list[float]:
    """Return offset + integer*interval times without accumulated timing error."""
    if sensor.sampling_start_offset_seconds > duration_seconds + EPSILON:
        return []
    count = math.floor(
        (duration_seconds - sensor.sampling_start_offset_seconds) / sensor.measurement_interval_seconds + EPSILON
    )
    return [
        round(sensor.sampling_start_offset_seconds + index * sensor.measurement_interval_seconds, 10)
        for index in range(count + 1)
    ]


def _stable_sensor_seed(base_seed: int, sensor_id: str) -> np.random.SeedSequence:
    """Derive a reproducible stream seed without Python's randomized hash()."""
    value = 2_166_136_261
    for byte in sensor_id.encode("utf-8"):
        value = ((value ^ byte) * 16_777_619) & 0xFFFFFFFF
    return np.random.SeedSequence([base_seed, value])


def _cartesian_sensor_specs(config: SimulationConfig) -> list[_ScheduledSensor]:
    assert config.position_sensor is not None
    sensors = [
        _ScheduledSensor(
            sensor_id=config.position_sensor.sensor_id,
            sensor_type="cartesian_position",
            enabled=config.position_sensor.enabled,
            measurement_interval_seconds=config.position_sensor.measurement_interval_seconds,
            sampling_start_offset_seconds=config.position_sensor.sampling_start_offset_seconds,
            outage_windows=config.position_sensor.outage_windows,
            sensor_position=(config.sensor_position_x, config.sensor_position_y),
            noise_std_x=config.position_sensor.noise_std_x,
            noise_std_y=config.position_sensor.noise_std_y,
            bias_x=config.position_sensor.bias_x,
            bias_y=config.position_sensor.bias_y,
            sensor_heading_degrees=0,
            range_noise_std_meters=0,
            bearing_noise_std_degrees=0,
        ),
        _ScheduledSensor(
            sensor_id=config.camera_sensor.sensor_id,
            sensor_type="cartesian_position",
            enabled=config.camera_sensor.enabled,
            measurement_interval_seconds=config.camera_sensor.measurement_interval_seconds,
            sampling_start_offset_seconds=config.camera_sensor.sampling_start_offset_seconds,
            outage_windows=config.camera_sensor.outage_windows,
            sensor_position=(0.0, 0.0),
            noise_std_x=config.camera_sensor.noise_std_x,
            noise_std_y=config.camera_sensor.noise_std_y,
            bias_x=config.camera_sensor.bias_x,
            bias_y=config.camera_sensor.bias_y,
            sensor_heading_degrees=0,
            range_noise_std_meters=0,
            bearing_noise_std_degrees=0,
        ),
    ]
    return [sensor for sensor in sensors if sensor.enabled]


def _radar_sensor_spec(config: SimulationConfig) -> _ScheduledSensor:
    return _ScheduledSensor(
        sensor_id=config.sensor_id,
        sensor_type="range_bearing",
        enabled=True,
        measurement_interval_seconds=config.sensor_interval_seconds,
        sampling_start_offset_seconds=0,
        outage_windows=config.outage_windows,
        sensor_position=(config.sensor_position_x, config.sensor_position_y),
        noise_std_x=0,
        noise_std_y=0,
        bias_x=0,
        bias_y=0,
        sensor_heading_degrees=config.sensor_heading_degrees,
        range_noise_std_meters=config.range_noise_std_meters,
        bearing_noise_std_degrees=config.bearing_noise_std_degrees,
    )


def _run_sensor_schedule(initial_target_state, config: SimulationConfig, sensor: _ScheduledSensor, rng: np.random.Generator) -> list:
    observations = []
    for measurement_time in _schedule_times(sensor, config.duration_seconds):
        measurement_state = advance_with_turns(initial_target_state, measurement_time, config.turn_events)
        # Consume this sensor's own random stream even when an outage omits the sample.
        observation = observe_sensor(
            measurement_state,
            rng=rng,
            sensor_type=sensor.sensor_type,
            sensor_id=sensor.sensor_id,
            sensor_position=sensor.sensor_position,
            noise_std_x=sensor.noise_std_x,
            noise_std_y=sensor.noise_std_y,
            bias_x=sensor.bias_x,
            bias_y=sensor.bias_y,
            sensor_heading_degrees=sensor.sensor_heading_degrees,
            range_noise_std_meters=sensor.range_noise_std_meters,
            bearing_noise_std_degrees=sensor.bearing_noise_std_degrees,
        )
        if observation is not None and not _is_sensor_outage(measurement_time, sensor.outage_windows):
            observations.append(observation)
    return observations


def run(config: SimulationConfig) -> SimulationResult:
    state = initial_state(config)
    truth = [state]
    while True:
        timestamp = state.timestamp
        if timestamp >= config.duration_seconds - EPSILON:
            break
        dt = min(config.simulation_timestep_seconds, config.duration_seconds - timestamp)
        state = advance_with_turns(state, dt, config.turn_events)
        if state.timestamp > truth[-1].timestamp + EPSILON:
            truth.append(state)

    initial_target_state = initial_state(config)
    if config.sensor_type == "range_bearing":
        sensors = [_radar_sensor_spec(config)]
        rngs = {sensors[0].sensor_id: np.random.default_rng(config.random_seed)}
    else:
        sensors = _cartesian_sensor_specs(config)
        assert config.position_sensor is not None
        # Keep the legacy position stream byte-for-byte compatible. The camera
        # stream is separately derived from a stable sensor-ID seed.
        rngs = {
            sensor.sensor_id: (
                np.random.default_rng(config.random_seed)
                if sensor.sensor_id == config.position_sensor.sensor_id
                else np.random.default_rng(_stable_sensor_seed(config.random_seed, sensor.sensor_id))
            )
            for sensor in sensors
        }

    observations = [
        observation
        for sensor in sensors
        for observation in _run_sensor_schedule(initial_target_state, config, sensor, rngs[sensor.sensor_id])
    ]
    observations.sort(key=lambda observation: (observation.measurement_timestamp, observation.sensor_id))

    tracker_config = TrackerConfig(
        initial_velocity_std_mps=config.tracker_initial_velocity_std_mps,
        acceleration_noise_spectral_density=config.tracker_acceleration_noise_spectral_density,
    )
    output_timestamps = [truth_state.timestamp for truth_state in truth]
    # The first milestone emits both Cartesian sensors, but keeps the existing
    # single-position tracking behavior until fusion consumes all sensor IDs.
    tracking_observations = (
        [observation for observation in observations if observation.sensor_id == config.position_sensor.sensor_id]
        if config.sensor_type == "cartesian_position" and config.position_sensor is not None
        else []
    )
    estimates = estimate_at_timestamps(tracking_observations, output_timestamps, tracker_config) if tracking_observations else []
    return SimulationResult(configuration=config, truth_history=truth, observations=observations, estimates=estimates)
