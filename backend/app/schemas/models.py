from typing import Literal
from pydantic import AliasChoices, BaseModel, Field, model_validator


class TurnEvent(BaseModel):
    start_time_seconds: float = Field(
        validation_alias=AliasChoices("start_time_seconds", "start_time"), ge=0
    )
    duration_seconds: float = Field(
        validation_alias=AliasChoices("duration_seconds", "duration"), gt=0
    )
    turn_rate_degrees_per_second: float = Field(
        validation_alias=AliasChoices(
            "turn_rate_degrees_per_second", "turn_rate", "turn_rate_deg_per_sec"
        )
    )

    @property
    def end_time_seconds(self) -> float:
        return self.start_time_seconds + self.duration_seconds


class SensorOutage(BaseModel):
    start_time_seconds: float = Field(
        validation_alias=AliasChoices("start_time_seconds", "start_time"), ge=0
    )
    end_time_seconds: float = Field(
        validation_alias=AliasChoices("end_time_seconds", "end_time"), gt=0
    )

    @model_validator(mode="after")
    def validate_window(self) -> "SensorOutage":
        if self.end_time_seconds <= self.start_time_seconds:
            raise ValueError("sensor outage end must be after its start")
        return self


class CartesianSensorConfig(BaseModel):
    """Configuration for a direct world-frame Cartesian position sensor."""

    sensor_id: str = Field(
        default="position-sensor-1",
        validation_alias=AliasChoices("sensor_id", "id"),
        min_length=1,
        max_length=80,
    )
    enabled: bool = True
    noise_std_x: float = Field(
        default=5,
        validation_alias=AliasChoices("noise_std_x", "measurement_noise_std_x", "measurement_sigma_x"),
        ge=0,
        le=1000,
    )
    noise_std_y: float = Field(
        default=5,
        validation_alias=AliasChoices("noise_std_y", "measurement_noise_std_y", "measurement_sigma_y"),
        ge=0,
        le=1000,
    )
    bias_x: float = Field(default=0, ge=-100_000, le=100_000)
    bias_y: float = Field(default=0, ge=-100_000, le=100_000)
    measurement_interval_seconds: float = Field(
        default=0.5,
        validation_alias=AliasChoices("measurement_interval_seconds", "sensor_interval_seconds"),
        gt=0,
        le=30,
    )
    sampling_start_offset_seconds: float = Field(
        default=0,
        validation_alias=AliasChoices("sampling_start_offset_seconds", "sampling_start_offset"),
        ge=0,
    )
    outage_windows: list[SensorOutage] = Field(default_factory=list)


class SimulationConfig(BaseModel):
    duration_seconds: float = Field(default=20, gt=0, le=300)
    simulation_timestep_seconds: float = Field(default=0.1, gt=0, le=1)
    sensor_interval_seconds: float = Field(default=0.5, gt=0, le=30)
    measurement_noise_std_x: float = Field(
        default=5,
        validation_alias=AliasChoices("measurement_noise_std_x", "measurement_sigma_x"),
        ge=0,
        le=1000,
    )
    measurement_noise_std_y: float = Field(
        default=5,
        validation_alias=AliasChoices("measurement_noise_std_y", "measurement_sigma_y"),
        ge=0,
        le=1000,
    )
    measurement_bias_x: float = Field(
        default=0,
        validation_alias=AliasChoices("measurement_bias_x", "bias_x"),
        ge=-100_000,
        le=100_000,
    )
    measurement_bias_y: float = Field(
        default=0,
        validation_alias=AliasChoices("measurement_bias_y", "bias_y"),
        ge=-100_000,
        le=100_000,
    )
    sensor_id: str = Field(default="position-sensor-1", min_length=1, max_length=80)
    sensor_type: Literal["cartesian_position", "range_bearing"] = "cartesian_position"
    sensor_position_x: float = Field(
        default=0,
        validation_alias=AliasChoices("sensor_position_x", "sensor_x"),
        ge=-100_000,
        le=100_000,
    )
    sensor_position_y: float = Field(
        default=0,
        validation_alias=AliasChoices("sensor_position_y", "sensor_y"),
        ge=-100_000,
        le=100_000,
    )
    sensor_heading_degrees: float = Field(default=0, ge=-360, le=360)
    range_noise_std_meters: float = Field(
        default=5,
        validation_alias=AliasChoices("range_noise_std_meters", "range_noise_std"),
        ge=0,
        le=1000,
    )
    bearing_noise_std_degrees: float = Field(default=2, ge=0, le=360)
    tracker_initial_velocity_std_mps: float = Field(default=10, ge=0, le=1000)
    tracker_acceleration_noise_spectral_density: float = Field(default=1, ge=0, le=1000)
    outage_windows: list[SensorOutage] = Field(default_factory=list)
    random_seed: int = Field(default=7, ge=0, le=2**31 - 1)
    initial_x: float = Field(default=0, ge=-100_000, le=100_000)
    initial_y: float = Field(default=0, ge=-100_000, le=100_000)
    initial_speed: float = Field(default=10, ge=0, le=1000)
    initial_heading_degrees: float = Field(default=0, ge=0, lt=360)
    turn_events: list[TurnEvent] = Field(default_factory=list)
    position_sensor: CartesianSensorConfig | None = None
    camera_sensor: CartesianSensorConfig = Field(
        default_factory=lambda: CartesianSensorConfig(sensor_id="camera-sensor-1", enabled=False)
    )

    @model_validator(mode="before")
    @classmethod
    def support_legacy_shared_noise(cls, values: object) -> object:
        if not isinstance(values, dict) or "measurement_noise_std" not in values:
            return values
        values = dict(values)
        legacy_noise = values.pop("measurement_noise_std")
        values.setdefault("measurement_noise_std_x", legacy_noise)
        values.setdefault("measurement_noise_std_y", legacy_noise)
        return values

    @model_validator(mode="after")
    def validate_turn_events(self) -> "SimulationConfig":
        if self.position_sensor is None:
            self.position_sensor = CartesianSensorConfig(
                sensor_id=self.sensor_id,
                enabled=self.sensor_type == "cartesian_position",
                noise_std_x=self.measurement_noise_std_x,
                noise_std_y=self.measurement_noise_std_y,
                bias_x=self.measurement_bias_x,
                bias_y=self.measurement_bias_y,
                measurement_interval_seconds=self.sensor_interval_seconds,
                outage_windows=list(self.outage_windows),
            )
        if self.position_sensor.sensor_id == self.camera_sensor.sensor_id:
            raise ValueError("position and camera sensor IDs must be distinct")
        if self.position_sensor.sampling_start_offset_seconds > self.duration_seconds:
            raise ValueError("position sensor sampling start offset must be within the simulation duration")
        if self.camera_sensor.sampling_start_offset_seconds > self.duration_seconds:
            raise ValueError("camera sensor sampling start offset must be within the simulation duration")
        events = sorted(self.turn_events, key=lambda event: event.start_time_seconds)
        previous_end = 0.0
        for event in events:
            if event.end_time_seconds > self.duration_seconds:
                raise ValueError("turn events must finish within the simulation duration")
            if event.start_time_seconds < previous_end:
                raise ValueError("turn events must not overlap")
            previous_end = event.end_time_seconds
        if self.sensor_type == "range_bearing" and (self.measurement_bias_x != 0 or self.measurement_bias_y != 0):
            raise ValueError("range/bearing sensors do not support Cartesian bias")
        for sensor_name, sensor in (("position", self.position_sensor), ("camera", self.camera_sensor)):
            outages = sorted(sensor.outage_windows, key=lambda window: window.start_time_seconds)
            previous_end = 0.0
            for window in outages:
                if window.end_time_seconds > self.duration_seconds:
                    raise ValueError(f"{sensor_name} sensor outage windows must finish within the simulation duration")
                if window.start_time_seconds < previous_end:
                    raise ValueError(f"{sensor_name} sensor outage windows must not overlap")
                previous_end = window.end_time_seconds
        return self

class TargetState(BaseModel):
    target_id: str
    timestamp: float
    x: float
    y: float
    vx: float
    vy: float

class SensorObservation(BaseModel):
    sensor_id: str
    measurement_timestamp: float
    availability_timestamp: float
    measurement_type: Literal["cartesian_position", "range_bearing"]
    measurement_values: list[float] = Field(min_length=2, max_length=2)
    measurement_covariance: list[list[float]]
    sensor_position: list[float] = Field(default_factory=lambda: [0.0, 0.0], min_length=2, max_length=2)

class TrackEstimate(BaseModel):
    track_id: str
    timestamp: float
    estimated_position: list[float]
    estimated_velocity: list[float]
    state_covariance: list[list[float]]
    last_measurement_timestamp: float | None = None
    measurement_age_seconds: float | None = None
    measurement_updated: bool = False


class TrackUpdateDiagnostic(BaseModel):
    sensor_id: str
    measurement_timestamp: float
    innovation: list[float] = Field(min_length=2, max_length=2)
    initialized: bool = False
    updated: bool = False


class SimulationResult(BaseModel):
    configuration: SimulationConfig
    truth_history: list[TargetState]
    observations: list[SensorObservation]
    estimates: list[TrackEstimate] = []
    comparison_estimates: dict[str, list[TrackEstimate]] = Field(default_factory=dict)
    update_diagnostics: list[TrackUpdateDiagnostic] = Field(default_factory=list)
