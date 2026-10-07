from fastapi import APIRouter
from app.schemas.models import SimulationConfig, SimulationResult
from app.simulation.runner import run

router = APIRouter(prefix="/api")

@router.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}

@router.post("/simulate", response_model=SimulationResult)
def simulate(config: SimulationConfig) -> SimulationResult:
    return run(config)
