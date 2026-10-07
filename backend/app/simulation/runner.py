import numpy as np
from app.schemas.models import SimulationConfig, SimulationResult
from app.simulation.motion import advance_with_turns
from app.simulation.sensors import observe
from app.simulation.world import initial_state
from app.tracking.kalman import estimate

def run(config: SimulationConfig) -> SimulationResult:
    rng = np.random.default_rng(config.random_seed)
    state = initial_state(config)
    truth = [state]
    observations = [observe(state, config.measurement_noise_std, rng)]
    next_sensor_time = round(config.sensor_interval_seconds, 10)
    while True:
        timestamp = state.timestamp
        if timestamp >= config.duration_seconds - 1e-9:
            break
        dt = min(config.simulation_timestep_seconds, config.duration_seconds - timestamp)
        end_time = timestamp + dt
        while next_sensor_time <= end_time + 1e-9:
            if next_sensor_time > timestamp + 1e-9:
                measurement_state = advance_with_turns(state, next_sensor_time - timestamp, config.turn_events)
                observations.append(observe(measurement_state, config.measurement_noise_std, rng))
            next_sensor_time = round(next_sensor_time + config.sensor_interval_seconds, 10)
        state = advance_with_turns(state, dt, config.turn_events)
        if state.timestamp > truth[-1].timestamp + 1e-9:
            truth.append(state)
    return SimulationResult(configuration=config, truth_history=truth, observations=observations, estimates=estimate(observations))
