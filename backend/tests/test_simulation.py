import pytest
from fastapi.testclient import TestClient
from app.main import app
from app.schemas.models import SimulationConfig
from app.simulation.runner import run
from app.simulation.world import velocity_from_speed_heading

def test_fixed_seed_is_deterministic():
    config = SimulationConfig(duration_seconds=2, random_seed=42)
    assert run(config).model_dump() == run(config).model_dump()

def test_constant_velocity_matches_analytic_position():
    result = run(SimulationConfig(duration_seconds=1, simulation_timestep_seconds=.25, initial_x=2, initial_y=-1, initial_speed=5, initial_heading_degrees=36.8698976458))
    last = result.truth_history[-1]
    assert last.x == pytest.approx(6); assert last.y == pytest.approx(2)

def test_heading_zero_moves_along_positive_x():
    vx, vy = velocity_from_speed_heading(10, 0)
    assert vx == pytest.approx(10)
    assert vy == pytest.approx(0)

def test_heading_90_moves_along_positive_y():
    vx, vy = velocity_from_speed_heading(10, 90)
    assert vx == pytest.approx(0, abs=1e-12)
    assert vy == pytest.approx(10)

def test_stationary_motion_is_valid():
    result = run(SimulationConfig(duration_seconds=2, initial_speed=0, initial_heading_degrees=90, initial_x=4, initial_y=-3))
    assert all(state.x == pytest.approx(4) and state.y == pytest.approx(-3) for state in result.truth_history)

def test_sensor_schedule_and_zero_noise():
    result = run(SimulationConfig(duration_seconds=2, simulation_timestep_seconds=.1, sensor_interval_seconds=.5, measurement_noise_std=0))
    assert [o.measurement_timestamp for o in result.observations] == pytest.approx([0, .5, 1, 1.5, 2])
    truth_by_time = {s.timestamp: s for s in result.truth_history}
    for observation in result.observations:
        state = truth_by_time[observation.measurement_timestamp]
        assert observation.measurement_values == pytest.approx([state.x, state.y])

def test_invalid_config_is_rejected():
    with pytest.raises(ValueError): SimulationConfig(duration_seconds=0)

def test_api():
    client = TestClient(app)
    assert client.get("/api/health").json() == {"status": "ok"}
    response = client.post("/api/simulate", json={"duration_seconds": 1})
    assert response.status_code == 200 and "truth_history" in response.json()
