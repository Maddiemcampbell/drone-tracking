import math

from app.schemas.models import SimulationConfig, TargetState


def velocity_from_speed_heading(speed: float, heading_degrees: float) -> tuple[float, float]:
    heading_radians = math.radians(heading_degrees)
    return speed * math.cos(heading_radians), speed * math.sin(heading_radians)


def initial_state(config: SimulationConfig) -> TargetState:
    vx, vy = velocity_from_speed_heading(config.initial_speed, config.initial_heading_degrees)
    return TargetState(target_id="target-1", timestamp=0, x=config.initial_x, y=config.initial_y, vx=vx, vy=vy)
