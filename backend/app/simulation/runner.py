import numpy as np
from app.schemas.models import SimulationConfig, SimulationResult
from app.simulation.motion import advance_with_turns
from app.simulation.sensors import observe_sensor
from app.simulation.world import initial_state
from app.tracking.kalman import TrackerConfig, estimate_at_timestamps

def run(config: SimulationConfig) -> SimulationResult:
    rng = np.random.default_rng(config.random_seed)
    state = initial_state(config)
    truth = [state]
    first_observation = observe_sensor(
            state,
            rng=rng,
            sensor_type=config.sensor_type,
            sensor_id=config.sensor_id,
            sensor_position=(config.sensor_position_x, config.sensor_position_y),
            noise_std_x=config.measurement_noise_std_x,
            noise_std_y=config.measurement_noise_std_y,
            bias_x=config.measurement_bias_x,
            bias_y=config.measurement_bias_y,
            sensor_heading_degrees=config.sensor_heading_degrees,
            range_noise_std_meters=config.range_noise_std_meters,
            bearing_noise_std_degrees=config.bearing_noise_std_degrees,
        )
    observations = [first_observation] if first_observation is not None else []
    sample_index = 1
    while True:
        timestamp = state.timestamp
        if timestamp >= config.duration_seconds - 1e-9:
            break
        dt = min(config.simulation_timestep_seconds, config.duration_seconds - timestamp)
        end_time = timestamp + dt
        while True:
            measurement_time = round(sample_index * config.sensor_interval_seconds, 10)
            if measurement_time > end_time + 1e-9:
                break
            if measurement_time > timestamp + 1e-9:
                measurement_state = advance_with_turns(state, measurement_time - timestamp, config.turn_events)
                observation = observe_sensor(
                        measurement_state,
                        rng=rng,
                        sensor_type=config.sensor_type,
                        sensor_id=config.sensor_id,
                        sensor_position=(config.sensor_position_x, config.sensor_position_y),
                        noise_std_x=config.measurement_noise_std_x,
                        noise_std_y=config.measurement_noise_std_y,
                        bias_x=config.measurement_bias_x,
                        bias_y=config.measurement_bias_y,
                        sensor_heading_degrees=config.sensor_heading_degrees,
                        range_noise_std_meters=config.range_noise_std_meters,
                        bearing_noise_std_degrees=config.bearing_noise_std_degrees,
                    )
                if observation is not None:
                    observations.append(observation)
            sample_index += 1
        state = advance_with_turns(state, dt, config.turn_events)
        if state.timestamp > truth[-1].timestamp + 1e-9:
            truth.append(state)
    tracker_config = TrackerConfig(
        initial_velocity_std_mps=config.tracker_initial_velocity_std_mps,
        acceleration_noise_spectral_density=config.tracker_acceleration_noise_spectral_density,
    )
    output_timestamps = [truth_state.timestamp for truth_state in truth]
    estimates = estimate_at_timestamps(observations, output_timestamps, tracker_config) if config.sensor_type == "cartesian_position" else []
    return SimulationResult(configuration=config, truth_history=truth, observations=observations, estimates=estimates)
