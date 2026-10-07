from app.schemas.models import TargetState

def advance(state: TargetState, dt: float) -> TargetState:
    return state.model_copy(update={"timestamp": round(state.timestamp + dt, 10), "x": state.x + state.vx * dt, "y": state.y + state.vy * dt})
