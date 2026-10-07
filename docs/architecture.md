# Architecture

The deterministic, fixed-step batch flow is:

```text
SimulationConfig -> World/Motion -> truth history
                              -> Position sensor -> observations
truth + observations -> API SimulationResult -> Canvas playback
observations -> ConstantVelocityKalmanFilter -> estimates
```

The tracker consumes `SensorObservation` only. It must never receive truth. Truth is retained solely for visualization and evaluation. The first tracker milestone is implemented in `tracking/kalman.py`, and its estimates are returned with the batch result and drawn as a dashed path.

`config.py` owns validation; `world.py` creates initial truth; `motion.py` propagates straight and analytic constant-turn-rate segments; `scenarios.py` provides editable examples; `sensors.py` creates noisy measurements; `runner.py` orchestrates; schemas define the typed contract; API routes validate and run batches. The frontend owns playback and converts meters to Canvas pixels.

Coordinates are local Cartesian meters: x increases rightward and y upward. Speed is meters per second. Heading is measured counterclockwise from positive x in degrees at the configuration boundary and converted to radians internally. Turn events use signed degrees per second: positive is counterclockwise and negative is clockwise. The integrator splits each fixed-step interval at event boundaries and uses an analytic constant-turn-rate update, with a straight-line fallback near zero turn rate. Timestamps are simulation seconds. The simulation timestep and sensor interval are separate. The Cartesian sensor has a stable ID and configurable world position, but reports world-frame x/y coordinates. Its x and y errors are independent zero-mean Gaussian samples with configured standard deviations σx and σy; covariance is diag(σx², σy²). Sampling occurs at integer schedule indices `k * sensor_interval_seconds` from k=0 through the run duration. Each truth state is evaluated at that exact timestamp, even between simulation ticks, and availability currently equals measurement time. Future radar range/bearing observations are documented rather than modeled as raw radar signals.
