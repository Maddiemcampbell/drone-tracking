# Architecture

The deterministic, fixed-step batch flow is:

```text
SimulationConfig -> World/Motion -> truth history
                              -> Sensor model -> observations
truth + observations -> API SimulationResult -> Canvas playback
observations -> ConstantVelocityKalmanFilter -> estimates
```

The tracker consumes `SensorObservation` only. It must never receive truth. Truth is retained solely for visualization and evaluation. The first tracker milestone is implemented in `tracking/kalman.py`, and its estimates are returned with the batch result and drawn as a dashed path.

`config.py` owns validation; `world.py` creates initial truth; `motion.py` propagates straight and analytic constant-turn-rate segments; `scenarios.py` provides editable examples; `sensors.py` creates noisy measurements; `runner.py` orchestrates; schemas define the typed contract; API routes validate and run batches. The frontend owns playback and converts meters to Canvas pixels. The sensor dispatcher selects the Cartesian position model or the simplified range/bearing model without adding a second scheduler.

Coordinates are local Cartesian meters: x increases rightward and y upward. Speed is meters per second. Heading is measured counterclockwise from positive x in degrees at the configuration boundary and converted to radians internally. Turn events use signed degrees per second: positive is counterclockwise and negative is clockwise. The integrator splits each fixed-step interval at event boundaries and uses an analytic constant-turn-rate update, with a straight-line fallback near zero turn rate. Timestamps are simulation seconds. The simulation timestep and sensor interval are separate. Both sensors have a stable ID and configurable world position. Sampling occurs at integer schedule indices `k * sensor_interval_seconds` from k=0 through the run duration. Each truth state is evaluated at that exact timestamp, even between simulation ticks, and availability currently equals measurement time.

The Cartesian sensor reports world-frame x/y as true position plus configurable constant bias and independent zero-mean Gaussian noise. Bias is not included in covariance: covariance is diag(σx², σy²) and describes random noise only. The simplified radar sensor reports `[range_meters, bearing_radians]`, where bearing is relative to the configured sensor heading and normalized to [-π, π). Its covariance is diag(σrange², σbearing²), with mixed units m² and rad²; bearing standard deviation is entered in degrees and converted to radians at the boundary. A target exactly at the sensor origin has undefined bearing and its sample is omitted. If additive range noise makes a range negative, that sample is omitted rather than clamped; this simple Gaussian model is limited near the sensor. There are no detection-range or field-of-view rules. The frontend converts valid radar observations to display coordinates, but the original range/bearing pair remains authoritative. Converted Cartesian errors can be correlated and vary with distance, so Cartesian sensor covariance must not be reused for radar.
