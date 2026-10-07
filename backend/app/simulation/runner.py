import numpy as np
from app.schemas.models import SimulationConfig, SimulationResult
from app.simulation.motion import advance
from app.simulation.sensors import observe
from app.simulation.world import initial_state
from app.tracking.kalman import estimate

def run(config: SimulationConfig) -> SimulationResult:
    rng = np.random.default_rng(config.random_seed)
    state = initial_state(config)
    truth, observations = [state], []
    next_sensor_time = 0.0
    steps = int(round(config.duration_seconds / config.simulation_timestep_seconds))
    for step in range(steps + 1):
        timestamp = round(step * config.simulation_timestep_seconds, 10)
        if timestamp + 1e-9 >= next_sensor_time:
            observations.append(observe(state, config.measurement_noise_std, rng))
            next_sensor_time = round(next_sensor_time + config.sensor_interval_seconds, 10)
        if step < steps:
            state = advance(state, config.simulation_timestep_seconds)
            truth.append(state)
    return SimulationResult(configuration=config, truth_history=truth, observations=observations, estimates=estimate(observations))
