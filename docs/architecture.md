# Architecture

The deterministic, fixed-step batch flow is:

```text
SimulationConfig -> World/Motion -> truth history
                              -> Position sensor -> observations
truth + observations -> API SimulationResult -> Canvas playback
observations -> ConstantVelocityKalmanFilter -> estimates
```

The tracker consumes `SensorObservation` only. It must never receive truth. Truth is retained solely for visualization and evaluation. The first tracker milestone is implemented in `tracking/kalman.py`, and its estimates are returned with the batch result and drawn as a dashed path.

`config.py` owns validation; `world.py` creates initial truth; `motion.py` propagates constant velocity; `sensors.py` creates noisy measurements; `runner.py` orchestrates; schemas define the typed contract; API routes validate and run batches. The frontend owns playback and converts meters to Canvas pixels.

Coordinates are local Cartesian meters: x increases rightward and y upward. Speed is meters per second. Heading is measured counterclockwise from positive x in degrees at the configuration boundary and converted to radians internally. Timestamps are simulation seconds. The simulation timestep and sensor interval are separate. The current sensor is deliberately simple: independent Gaussian noise and diagonal covariance for Cartesian position. Future radar range/bearing observations are documented rather than modeled as raw radar signals.
