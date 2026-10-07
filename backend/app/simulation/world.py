from app.schemas.models import SimulationConfig, TargetState

def initial_state(config: SimulationConfig) -> TargetState:
    return TargetState(target_id="target-1", timestamp=0, x=config.initial_x, y=config.initial_y, vx=config.initial_vx, vy=config.initial_vy)
