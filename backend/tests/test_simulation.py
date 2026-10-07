import math

import numpy as np
import pytest
from fastapi.testclient import TestClient
from app.main import app
from app.schemas.models import CartesianSensorConfig, SensorOutage, SimulationConfig, TargetState, TurnEvent
from app.simulation.motion import advance_with_turns
from app.simulation.scenarios import example_scenarios
from app.simulation.runner import run
from app.simulation.sensors import normalize_angle_radians, observe_range_bearing
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
    assert set(scenarios) == {"straight_flight", "gradual_90_degree_turn", "s_shaped_path", "straight_with_outage", "turn_with_measurements", "turn_during_outage"}
    assert scenarios["straight_flight"].turn_events == []
    assert len(scenarios["gradual_90_degree_turn"].turn_events) == 1
    assert len(scenarios["s_shaped_path"].turn_events) == 2
    assert scenarios["straight_with_outage"].outage_windows[0].start_time_seconds == 5
    assert len(scenarios["turn_during_outage"].outage_windows) == 1


def test_outage_windows_use_half_open_boundaries_and_are_validated():
    result = run(SimulationConfig(duration_seconds=2.5, sensor_interval_seconds=.5, measurement_noise_std=0, outage_windows=[SensorOutage(start_time_seconds=1, end_time_seconds=2)]))
    assert [observation.measurement_timestamp for observation in result.observations] == pytest.approx([0, .5, 2, 2.5])
    with pytest.raises(ValueError, match="after"):
        SimulationConfig(duration_seconds=3, outage_windows=[SensorOutage(start_time_seconds=1, end_time_seconds=1)])
    with pytest.raises(ValueError, match="finish"):
        SimulationConfig(duration_seconds=2, outage_windows=[SensorOutage(start_time_seconds=1, end_time_seconds=3)])
    with pytest.raises(ValueError, match="overlap"):
        SimulationConfig(duration_seconds=4, outage_windows=[SensorOutage(start_time_seconds=1, end_time_seconds=3), SensorOutage(start_time_seconds=2, end_time_seconds=4)])


def test_outage_preserves_seeded_noise_for_unaffected_samples_and_truth():
    base = run(SimulationConfig(duration_seconds=3, sensor_interval_seconds=.5, random_seed=22, measurement_noise_std_x=3, measurement_noise_std_y=4))
    outage = run(SimulationConfig(duration_seconds=3, sensor_interval_seconds=.5, random_seed=22, measurement_noise_std_x=3, measurement_noise_std_y=4, outage_windows=[SensorOutage(start_time_seconds=1, end_time_seconds=2)]))
    assert outage.truth_history == base.truth_history
    base_by_time = {observation.measurement_timestamp: observation for observation in base.observations}
    for observation in outage.observations:
        if observation.measurement_timestamp < 1 or observation.measurement_timestamp >= 2:
            assert observation.measurement_values == pytest.approx(base_by_time[observation.measurement_timestamp].measurement_values)


def test_tracker_predicts_through_outage_and_reports_measurement_age():
    result = run(SimulationConfig(duration_seconds=2, simulation_timestep_seconds=.5, sensor_interval_seconds=.5, random_seed=3, measurement_noise_std_x=1, measurement_noise_std_y=1, tracker_acceleration_noise_spectral_density=2, outage_windows=[SensorOutage(start_time_seconds=1, end_time_seconds=2)]))
    assert [estimate.timestamp for estimate in result.estimates] == pytest.approx([0, .5, 1, 1.5, 2])
    assert [estimate.measurement_updated for estimate in result.estimates] == [True, True, False, False, True]
    assert result.estimates[2].last_measurement_timestamp == pytest.approx(.5)
    assert result.estimates[2].measurement_age_seconds == pytest.approx(.5)
    assert result.estimates[3].measurement_age_seconds == pytest.approx(1)
    assert np.trace(np.asarray(result.estimates[3].state_covariance)) > np.trace(np.asarray(result.estimates[2].state_covariance))
    assert result.estimates[4].measurement_updated is True


def test_outage_before_first_observation_leaves_tracker_uninitialized():
    result = run(SimulationConfig(duration_seconds=2, sensor_interval_seconds=.75, outage_windows=[SensorOutage(start_time_seconds=0, end_time_seconds=2)]))
    assert result.observations == []
    assert result.estimates == []

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

def test_zero_noise_with_bias_produces_configured_offset_without_changing_truth():
    config = SimulationConfig(duration_seconds=1, sensor_interval_seconds=.25, measurement_noise_std_x=0, measurement_noise_std_y=0, measurement_bias_x=3.5, measurement_bias_y=-2)
    result = run(config)
    for observation in result.observations:
        truth_x = 10 * observation.measurement_timestamp
        truth_y = 0
        assert observation.measurement_values == pytest.approx([truth_x + 3.5, truth_y - 2])
        assert observation.measurement_covariance == [[0, 0], [0, 0]]
    assert result.truth_history[-1].x == pytest.approx(10)
    assert result.truth_history[-1].y == pytest.approx(0)

def test_zero_noise_zero_bias_has_zero_position_error():
    config = SimulationConfig(duration_seconds=1, sensor_interval_seconds=.25, measurement_noise_std_x=0, measurement_noise_std_y=0, measurement_bias_x=0, measurement_bias_y=0)
    result = run(config)
    for observation in result.observations:
        assert observation.measurement_values == pytest.approx([10 * observation.measurement_timestamp, 0])

def test_bias_does_not_change_motion_or_random_noise_covariance():
    unbiased = run(SimulationConfig(duration_seconds=1, random_seed=4, measurement_noise_std_x=2, measurement_noise_std_y=3))
    biased = run(SimulationConfig(duration_seconds=1, random_seed=4, measurement_noise_std_x=2, measurement_noise_std_y=3, measurement_bias_x=7, measurement_bias_y=-4))
    assert unbiased.truth_history == biased.truth_history
    for first, second in zip(unbiased.observations, biased.observations):
        assert second.measurement_values == pytest.approx([first.measurement_values[0] + 7, first.measurement_values[1] - 4])
        assert second.measurement_covariance == first.measurement_covariance

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


def test_radar_mode_remains_sensor_only_without_tracker_estimates():
    result = run(SimulationConfig(sensor_type="range_bearing", duration_seconds=1, range_noise_std_meters=0, bearing_noise_std_degrees=0, sensor_position_y=-1))
    assert result.observations
    assert result.estimates == []


def test_runner_emits_output_timestamps_and_updates_before_same_time_output():
    result = run(SimulationConfig(duration_seconds=1, simulation_timestep_seconds=.5, sensor_interval_seconds=.5, measurement_noise_std_x=0, measurement_noise_std_y=0, tracker_acceleration_noise_spectral_density=0))
    assert [estimate.timestamp for estimate in result.estimates] == pytest.approx([0, .5, 1])
    assert [estimate.measurement_updated for estimate in result.estimates] == [True, True, True]
    assert result.estimates[0].last_measurement_timestamp == pytest.approx(0)


def test_runner_predicts_without_update_when_no_measurement_occurs():
    result = run(SimulationConfig(duration_seconds=1.5, simulation_timestep_seconds=.5, sensor_interval_seconds=.75, measurement_noise_std_x=0, measurement_noise_std_y=0, tracker_acceleration_noise_spectral_density=0))
    assert [estimate.timestamp for estimate in result.estimates] == pytest.approx([0, .5, 1, 1.5])
    assert [estimate.measurement_updated for estimate in result.estimates] == [True, False, False, True]
    assert result.estimates[2].last_measurement_timestamp == pytest.approx(.75)


def test_runner_handles_irregular_measurement_intervals_causally():
    result = run(SimulationConfig(duration_seconds=2, simulation_timestep_seconds=.1, sensor_interval_seconds=.7, measurement_noise_std_x=2, measurement_noise_std_y=2))
    assert len(result.estimates) == len(result.truth_history)
    assert [observation.measurement_timestamp for observation in result.observations] == pytest.approx([0, .7, 1.4])
    update_times = [estimate.timestamp for estimate in result.estimates if estimate.measurement_updated]
    assert update_times == pytest.approx([0, .7, 1.4])


def test_seeded_tracker_position_rmse_beats_raw_observations_after_warmup():
    result = run(SimulationConfig(duration_seconds=10, simulation_timestep_seconds=.1, sensor_interval_seconds=.5, measurement_noise_std_x=5, measurement_noise_std_y=5, tracker_initial_velocity_std_mps=10, tracker_acceleration_noise_spectral_density=1, random_seed=7))
    truth_by_time = {round(state.timestamp, 10): state for state in result.truth_history}
    estimates_by_time = {round(estimate.timestamp, 10): estimate for estimate in result.estimates}
    observations_by_time = {round(observation.measurement_timestamp, 10): observation for observation in result.observations}
    warm_measurement_times = sorted(timestamp for timestamp in observations_by_time if timestamp >= 2)
    raw_squared_errors = []
    track_squared_errors = []
    for timestamp in warm_measurement_times:
        truth = truth_by_time[timestamp]
        observation = observations_by_time[timestamp]
        estimate = estimates_by_time[timestamp]
        raw_squared_errors.append((observation.measurement_values[0] - truth.x) ** 2 + (observation.measurement_values[1] - truth.y) ** 2)
        track_squared_errors.append((estimate.estimated_position[0] - truth.x) ** 2 + (estimate.estimated_position[1] - truth.y) ** 2)
    raw_rmse = math.sqrt(sum(raw_squared_errors) / len(raw_squared_errors))
    track_rmse = math.sqrt(sum(track_squared_errors) / len(track_squared_errors))
    assert track_rmse < raw_rmse


def test_radar_axis_targets_use_sensor_relative_range_and_bearing():
    rng = np.random.default_rng(1)
    east = observe_range_bearing(TargetState(target_id="drone", timestamp=0, x=10, y=0, vx=0, vy=0), rng, sensor_id="radar", sensor_position=(0, 0), sensor_heading_degrees=0, range_noise_std_meters=0, bearing_noise_std_degrees=0)
    north = observe_range_bearing(TargetState(target_id="drone", timestamp=0, x=0, y=10, vx=0, vy=0), rng, sensor_id="radar", sensor_position=(0, 0), sensor_heading_degrees=0, range_noise_std_meters=0, bearing_noise_std_degrees=0)
    assert east is not None and east.measurement_values == pytest.approx([10, 0])
    assert north is not None and north.measurement_values == pytest.approx([10, math.pi / 2])


def test_radar_sensor_position_and_heading_change_local_measurement():
    state = TargetState(target_id="drone", timestamp=2, x=5, y=10, vx=0, vy=0)
    observation = observe_range_bearing(state, np.random.default_rng(2), sensor_id="radar", sensor_position=(5, 5), sensor_heading_degrees=90, range_noise_std_meters=0, bearing_noise_std_degrees=0)
    assert observation is not None
    assert observation.measurement_values == pytest.approx([5, 0])


def test_radar_zero_noise_converts_back_to_world_position_and_covariance_is_mixed_units():
    config = SimulationConfig(sensor_type="range_bearing", duration_seconds=1, initial_x=3, initial_y=4, initial_speed=0, sensor_position_x=1, sensor_position_y=1, sensor_heading_degrees=30, range_noise_std_meters=0, bearing_noise_std_degrees=0)
    result = run(config)
    observation = result.observations[0]
    measured_range, measured_bearing = observation.measurement_values
    world_angle = math.radians(config.sensor_heading_degrees) + measured_bearing
    assert [config.sensor_position_x + measured_range * math.cos(world_angle), config.sensor_position_y + measured_range * math.sin(world_angle)] == pytest.approx([3, 4])
    assert observation.measurement_covariance == [[0, 0], [0, 0]]
    noisy_config = config.model_copy(update={"bearing_noise_std_degrees": 90})
    noisy_observation = run(noisy_config).observations[0]
    assert noisy_observation.measurement_covariance[0] == pytest.approx([0, 0])
    assert noisy_observation.measurement_covariance[1] == pytest.approx([0, (math.pi / 2) ** 2])


def test_radar_bearing_wraps_to_minus_pi_inclusive_boundary():
    target = TargetState(target_id="drone", timestamp=0, x=math.cos(math.radians(179)), y=math.sin(math.radians(179)), vx=0, vy=0)
    observation = observe_range_bearing(target, np.random.default_rng(3), sensor_id="radar", sensor_position=(0, 0), sensor_heading_degrees=-179, range_noise_std_meters=0, bearing_noise_std_degrees=0)
    assert observation is not None
    assert observation.measurement_values[1] == pytest.approx(math.radians(-2))
    assert normalize_angle_radians(math.pi) == pytest.approx(-math.pi)


def test_radar_measurements_during_turn_use_exact_timestamp_and_preserve_timestamps():
    config = SimulationConfig(sensor_type="range_bearing", duration_seconds=1, simulation_timestep_seconds=.1, sensor_interval_seconds=.25, range_noise_std_meters=0, bearing_noise_std_degrees=0, sensor_position_y=-10, initial_speed=10, turn_events=[TurnEvent(start_time_seconds=.2, duration_seconds=.4, turn_rate_degrees_per_second=90)])
    result = run(config)
    expected = advance_with_turns(initial_state(config), .25, config.turn_events)
    observation = result.observations[1]
    expected_range = math.hypot(expected.x, expected.y + 10)
    expected_bearing = math.atan2(expected.y + 10, expected.x)
    assert observation.measurement_timestamp == pytest.approx(.25)
    assert observation.availability_timestamp == pytest.approx(.25)
    assert observation.measurement_values == pytest.approx([expected_range, expected_bearing])


def test_radar_origin_and_negative_noisy_range_samples_are_omitted():
    origin = TargetState(target_id="drone", timestamp=0, x=2, y=-1, vx=0, vy=0)
    assert observe_range_bearing(origin, np.random.default_rng(4), sensor_id="radar", sensor_position=(2, -1), sensor_heading_degrees=0, range_noise_std_meters=0, bearing_noise_std_degrees=0) is None

    class NegativeRangeRng:
        def normal(self, _mean, _std):
            return -2.0

    assert observe_range_bearing(TargetState(target_id="drone", timestamp=0, x=1, y=0, vx=0, vy=0), NegativeRangeRng(), sensor_id="radar", sensor_position=(0, 0), sensor_heading_degrees=0, range_noise_std_meters=1, bearing_noise_std_degrees=0) is None


def test_radar_configuration_rejects_negative_noise_interval_and_cartesian_bias():
    with pytest.raises(ValueError):
        SimulationConfig(sensor_type="range_bearing", range_noise_std_meters=-1)
    with pytest.raises(ValueError):
        SimulationConfig(sensor_type="range_bearing", bearing_noise_std_degrees=-1)
    with pytest.raises(ValueError):
        SimulationConfig(sensor_type="range_bearing", sensor_interval_seconds=0)
    with pytest.raises(ValueError, match="bias"):
        SimulationConfig(sensor_type="range_bearing", measurement_bias_x=1)


def _two_cartesian_sensor_config(**updates) -> SimulationConfig:
    config = SimulationConfig(
        duration_seconds=2,
        simulation_timestep_seconds=.1,
        initial_speed=8,
        position_sensor=CartesianSensorConfig(
            sensor_id="position-A",
            enabled=True,
            noise_std_x=0,
            noise_std_y=0,
            measurement_interval_seconds=.5,
        ),
        camera_sensor=CartesianSensorConfig(
            sensor_id="camera-B",
            enabled=True,
            noise_std_x=0,
            noise_std_y=0,
            measurement_interval_seconds=.75,
            sampling_start_offset_seconds=.25,
        ),
    )
    return config.model_copy(update=updates)


def test_two_cartesian_sensors_follow_independent_sampling_schedules():
    result = run(_two_cartesian_sensor_config())
    by_sensor = {
        sensor_id: [observation.measurement_timestamp for observation in result.observations if observation.sensor_id == sensor_id]
        for sensor_id in ("position-A", "camera-B")
    }
    assert by_sensor["position-A"] == pytest.approx([0, .5, 1, 1.5, 2])
    assert by_sensor["camera-B"] == pytest.approx([.25, 1, 1.75])
    assert [(observation.measurement_timestamp, observation.sensor_id) for observation in result.observations] == [
        (0, "position-A"),
        (.25, "camera-B"),
        (.5, "position-A"),
        (1, "camera-B"),
        (1, "position-A"),
        (1.5, "position-A"),
        (1.75, "camera-B"),
        (2, "position-A"),
    ]


def test_two_cartesian_sensors_measure_the_same_world_coordinates():
    result = run(_two_cartesian_sensor_config())
    truth_by_time = {round(state.timestamp, 10): state for state in result.truth_history}
    for observation in result.observations:
        truth = advance_with_turns(initial_state(result.configuration), observation.measurement_timestamp, result.configuration.turn_events)
        assert observation.measurement_values == pytest.approx([truth.x, truth.y])
        assert not hasattr(observation, "target_id")


def test_sensor_streams_are_deterministic_and_disabling_camera_preserves_position_samples():
    both_config = _two_cartesian_sensor_config(random_seed=17)
    both = run(both_config)
    repeated = run(both_config)
    assert both.model_dump() == repeated.model_dump()
    camera_disabled = both_config.model_copy(update={"camera_sensor": both_config.camera_sensor.model_copy(update={"enabled": False})})
    position_only = run(camera_disabled)
    both_position = [observation for observation in both.observations if observation.sensor_id == "position-A"]
    assert [observation.model_dump() for observation in both_position] == [observation.model_dump() for observation in position_only.observations]


def test_bias_and_outage_only_affect_the_configured_sensor():
    base = _two_cartesian_sensor_config(
        position_sensor=CartesianSensorConfig(
            sensor_id="position-A", enabled=True, noise_std_x=0, noise_std_y=0, measurement_interval_seconds=.5
        ),
        camera_sensor=CartesianSensorConfig(
            sensor_id="camera-B", enabled=True, noise_std_x=0, noise_std_y=0, measurement_interval_seconds=.5
        ),
    )
    changed_position = base.model_copy(update={
        "position_sensor": base.position_sensor.model_copy(update={
            "bias_x": 3,
            "bias_y": -2,
            "outage_windows": [SensorOutage(start_time_seconds=.5, end_time_seconds=1.5)],
        })
    })
    base_result = run(base)
    changed_result = run(changed_position)
    base_camera = [observation for observation in base_result.observations if observation.sensor_id == "camera-B"]
    changed_camera = [observation for observation in changed_result.observations if observation.sensor_id == "camera-B"]
    assert [observation.model_dump() for observation in base_camera] == [observation.model_dump() for observation in changed_camera]
    assert [observation.measurement_timestamp for observation in changed_result.observations if observation.sensor_id == "position-A"] == pytest.approx([0, 1.5, 2])
    changed_position_first = next(observation for observation in changed_result.observations if observation.sensor_id == "position-A")
    assert changed_position_first.measurement_values == pytest.approx([3, -2])
