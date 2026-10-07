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

class SimulationConfig(BaseModel):
    duration_seconds: float = Field(default=20, gt=0, le=300)
    simulation_timestep_seconds: float = Field(default=0.1, gt=0, le=1)
    sensor_interval_seconds: float = Field(default=0.5, gt=0, le=30)
    measurement_noise_std: float = Field(default=5, ge=0, le=1000)
    random_seed: int = Field(default=7, ge=0, le=2**31 - 1)
    initial_x: float = Field(default=0, ge=-100_000, le=100_000)
    initial_y: float = Field(default=0, ge=-100_000, le=100_000)
    initial_speed: float = Field(default=10, ge=0, le=1000)
    initial_heading_degrees: float = Field(default=0, ge=0, lt=360)
    turn_events: list[TurnEvent] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_turn_events(self) -> "SimulationConfig":
        events = sorted(self.turn_events, key=lambda event: event.start_time_seconds)
        previous_end = 0.0
        for event in events:
            if event.end_time_seconds > self.duration_seconds:
                raise ValueError("turn events must finish within the simulation duration")
            if event.start_time_seconds < previous_end:
                raise ValueError("turn events must not overlap")
            previous_end = event.end_time_seconds
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
    measurement_type: Literal["cartesian_position"]
    measurement_values: list[float] = Field(min_length=2, max_length=2)
    measurement_covariance: list[list[float]]

class TrackEstimate(BaseModel):
    track_id: str
    timestamp: float
    estimated_position: list[float]
    estimated_velocity: list[float]
    state_covariance: list[list[float]]

class SimulationResult(BaseModel):
    configuration: SimulationConfig
    truth_history: list[TargetState]
    observations: list[SensorObservation]
    estimates: list[TrackEstimate] = []
