from typing import Literal
from pydantic import BaseModel, Field

class SimulationConfig(BaseModel):
    duration_seconds: float = Field(default=20, gt=0, le=300)
    simulation_timestep_seconds: float = Field(default=0.1, gt=0, le=1)
    sensor_interval_seconds: float = Field(default=0.5, gt=0, le=30)
    measurement_noise_std: float = Field(default=5, ge=0, le=1000)
    random_seed: int = Field(default=7, ge=0, le=2**31 - 1)
    initial_x: float = Field(default=0, ge=-100_000, le=100_000)
    initial_y: float = Field(default=0, ge=-100_000, le=100_000)
    initial_vx: float = Field(default=10, ge=-1000, le=1000)
    initial_vy: float = Field(default=5, ge=-1000, le=1000)

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
