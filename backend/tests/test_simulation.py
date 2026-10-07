import math

import pytest
from fastapi.testclient import TestClient
from app.main import app
from app.schemas.models import SimulationConfig, TurnEvent
from app.simulation.motion import advance_with_turns
from app.simulation.scenarios import example_scenarios
from app.simulation.runner import run
from app.simulation.world import initial_state, velocity_from_speed_heading

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

def test_turn_keeps_speed_constant_and_changes_heading_by_rate_times_duration():
    config = SimulationConfig(initial_speed=10, initial_heading_degrees=0, duration_seconds=2, turn_events=[TurnEvent(start_time_seconds=0, duration_seconds=2, turn_rate_degrees_per_second=45)])
    state = advance_with_turns(initial_state(config), 2, config.turn_events)
    assert (state.vx**2 + state.vy**2) ** 0.5 == pytest.approx(10)
    assert math.degrees(math.atan2(state.vy, state.vx)) == pytest.approx(90)

    state = initial_state(config)
    for _ in range(8):
        state = advance_with_turns(state, 0.25, config.turn_events)
        assert math.hypot(state.vx, state.vy) == pytest.approx(10)

def test_turn_position_is_continuous_at_boundaries():
    config = SimulationConfig(duration_seconds=4, turn_events=[TurnEvent(start_time_seconds=1, duration_seconds=2, turn_rate_degrees_per_second=45)])
    start = initial_state(config)
    at_boundary = advance_with_turns(start, 1, config.turn_events)
    after_boundary = advance_with_turns(at_boundary, 0.000001, config.turn_events)
    assert after_boundary.x == pytest.approx(at_boundary.x + at_boundary.vx * 0.000001, abs=1e-8)
    assert after_boundary.y == pytest.approx(at_boundary.y + at_boundary.vy * 0.000001, abs=1e-8)

def test_full_circle_returns_to_start():
    config = SimulationConfig(duration_seconds=4, turn_events=[TurnEvent(start_time_seconds=0, duration_seconds=4, turn_rate_degrees_per_second=90)])
    start = initial_state(config)
    final = advance_with_turns(start, 4, config.turn_events)
    assert final.x == pytest.approx(start.x, abs=1e-8)
    assert final.y == pytest.approx(start.y, abs=1e-8)
    assert final.vx == pytest.approx(start.vx, abs=1e-8)
    assert final.vy == pytest.approx(start.vy, abs=1e-8)

def test_turn_boundaries_between_ticks_are_integrated_exactly():
    config = SimulationConfig(duration_seconds=1.2, simulation_timestep_seconds=.5, turn_events=[TurnEvent(start_time_seconds=.25, duration_seconds=.5, turn_rate_degrees_per_second=90)])
    result = run(config)
    expected = advance_with_turns(initial_state(config), 1.2, config.turn_events)
    assert result.truth_history[-1].x == pytest.approx(expected.x)
    assert result.truth_history[-1].y == pytest.approx(expected.y)
    assert result.truth_history[-1].vx == pytest.approx(expected.vx)
    assert result.truth_history[-1].vy == pytest.approx(expected.vy)

def test_overlapping_and_invalid_turns_are_rejected():
    with pytest.raises(ValueError):
        SimulationConfig(duration_seconds=4, turn_events=[TurnEvent(start_time_seconds=1, duration_seconds=0, turn_rate_degrees_per_second=10)])
    with pytest.raises(ValueError, match="overlap"):
        SimulationConfig(duration_seconds=4, turn_events=[TurnEvent(start_time_seconds=1, duration_seconds=2, turn_rate_degrees_per_second=10), TurnEvent(start_time_seconds=2, duration_seconds=1, turn_rate_degrees_per_second=-10)])
    with pytest.raises(ValueError, match="finish"):
        SimulationConfig(duration_seconds=2, turn_events=[TurnEvent(start_time_seconds=1, duration_seconds=2, turn_rate_degrees_per_second=10)])

def test_example_scenarios_are_available():
    scenarios = example_scenarios()
    assert set(scenarios) == {"straight_flight", "gradual_90_degree_turn", "s_shaped_path"}
    assert scenarios["straight_flight"].turn_events == []
    assert len(scenarios["gradual_90_degree_turn"].turn_events) == 1
    assert len(scenarios["s_shaped_path"].turn_events) == 2

def test_sensor_schedule_and_zero_noise():
    result = run(SimulationConfig(duration_seconds=2, simulation_timestep_seconds=.1, sensor_interval_seconds=.5, measurement_noise_std=0))
    assert [o.measurement_timestamp for o in result.observations] == pytest.approx([0, .5, 1, 1.5, 2])
    truth_by_time = {s.timestamp: s for s in result.truth_history}
    for observation in result.observations:
        state = truth_by_time[observation.measurement_timestamp]
        assert observation.measurement_values == pytest.approx([state.x, state.y])

def test_between_tick_measurements_use_exact_truth_timestamp():
    config = SimulationConfig(duration_seconds=1, simulation_timestep_seconds=.1, sensor_interval_seconds=.25, measurement_noise_std=0, initial_speed=8, initial_heading_degrees=0)
    result = run(config)
    assert [observation.measurement_timestamp for observation in result.observations] == pytest.approx([0, .25, .5, .75, 1])
    for observation in result.observations:
        assert observation.measurement_values == pytest.approx([8 * observation.measurement_timestamp, 0])

@pytest.mark.parametrize(("interval", "expected_times"), [
    (.25, [0, .25, .5, .75, 1]),
    (.3, [0, .3, .6, .9]),
    (2, [0]),
])
def test_sample_schedule_uses_integer_indices(interval, expected_times):
    result = run(SimulationConfig(duration_seconds=1, sensor_interval_seconds=interval, measurement_noise_std=0))
    assert [observation.measurement_timestamp for observation in result.observations] == pytest.approx(expected_times)

def test_measurement_during_turn_uses_exact_motion_time():
    config = SimulationConfig(duration_seconds=1, simulation_timestep_seconds=.1, sensor_interval_seconds=.25, measurement_noise_std_x=0, measurement_noise_std_y=0, initial_speed=10, turn_events=[TurnEvent(start_time_seconds=.2, duration_seconds=.4, turn_rate_degrees_per_second=90)])
    result = run(config)
    expected = advance_with_turns(initial_state(config), .25, config.turn_events)
    assert result.observations[1].measurement_timestamp == pytest.approx(.25)
    assert result.observations[1].measurement_values == pytest.approx([expected.x, expected.y])

def test_sensor_metadata_and_covariance_are_configurable():
    result = run(SimulationConfig(duration_seconds=1, measurement_noise_std_x=2, measurement_noise_std_y=3, sensor_id="roof-sensor", sensor_position_x=12, sensor_position_y=-4))
    observation = result.observations[0]
    assert observation.sensor_id == "roof-sensor"
    assert observation.sensor_position == pytest.approx([12, -4])
    assert observation.measurement_covariance == [[4, 0], [0, 9]]

def test_seed_changes_observations_but_not_truth():
    first = run(SimulationConfig(duration_seconds=2, random_seed=1))
    second = run(SimulationConfig(duration_seconds=2, random_seed=2))
    assert first.truth_history == second.truth_history
    assert [observation.measurement_values for observation in first.observations] != [observation.measurement_values for observation in second.observations]

def test_sensor_interval_changes_timing_but_not_truth():
    first = run(SimulationConfig(duration_seconds=2, sensor_interval_seconds=.25))
    second = run(SimulationConfig(duration_seconds=2, sensor_interval_seconds=.75))
    assert first.truth_history == second.truth_history
    assert [observation.measurement_timestamp for observation in first.observations] != [observation.measurement_timestamp for observation in second.observations]

def test_invalid_config_is_rejected():
    with pytest.raises(ValueError): SimulationConfig(duration_seconds=0)
    with pytest.raises(ValueError): SimulationConfig(sensor_interval_seconds=0)
    with pytest.raises(ValueError): SimulationConfig(measurement_noise_std_x=-1)
    with pytest.raises(ValueError): SimulationConfig(measurement_noise_std_y=-1)

def test_api():
    client = TestClient(app)
    assert client.get("/api/health").json() == {"status": "ok"}
    response = client.post("/api/simulate", json={"duration_seconds": 1})
    assert response.status_code == 200 and "truth_history" in response.json()
