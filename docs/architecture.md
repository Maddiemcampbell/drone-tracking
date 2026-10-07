# Architecture

The deterministic, fixed-step batch flow is:

```text
SimulationConfig -> World/Motion -> truth history
                              -> Sensor model -> observations
truth + observations -> API SimulationResult -> Canvas playback
observations -> ConstantVelocityKalmanFilter -> estimates
truth + observations + estimates -> evaluation-only metrics
```

The tracker consumes `SensorObservation` only. It must never receive truth, true initial velocity, or motion-event configuration. Truth is retained solely for visualization and evaluation. The tracker initializes from the first Cartesian observation, merges measurement and output events chronologically, predicts to each event time, and applies causal Cartesian measurement updates. The frontend renders only estimates emitted by that causal schedule at or before the playback time; it never interpolates toward a future estimate.

`config.py` owns validation; `world.py` creates initial truth; `motion.py` propagates straight and analytic constant-turn-rate segments; `scenarios.py` provides editable examples; `sensors.py` creates noisy measurements; `runner.py` orchestrates; schemas define the typed contract; API routes validate and run batches. The frontend owns playback and converts meters to Canvas pixels. The sensor dispatcher selects the Cartesian position model or the simplified range/bearing model without adding a second scheduler.

Coordinates are local Cartesian meters: x increases rightward and y upward. Speed is meters per second. Heading is measured counterclockwise from positive x in degrees at the configuration boundary and converted to radians internally. Turn events use signed degrees per second: positive is counterclockwise and negative is clockwise. The integrator splits each fixed-step interval at event boundaries and uses an analytic constant-turn-rate update, with a straight-line fallback near zero turn rate. Timestamps are simulation seconds. The simulation timestep and sensor interval are separate. Both sensors have a stable ID and configurable world position. Sampling occurs at integer schedule indices `k * sensor_interval_seconds` from k=0 through the run duration. Each truth state is evaluated at that exact timestamp, even between simulation ticks, and availability currently equals measurement time.

The Cartesian sensor reports world-frame x/y as true position plus configurable constant bias and independent zero-mean Gaussian noise. Bias is not included in covariance: covariance is diag(σx², σy²) and describes random noise only. The simplified radar sensor reports `[range_meters, bearing_radians]`, where bearing is relative to the configured sensor heading and normalized to [-π, π). Its covariance is diag(σrange², σbearing²), with mixed units m² and rad²; bearing standard deviation is entered in degrees and converted to radians at the boundary. A target exactly at the sensor origin has undefined bearing and its sample is omitted. If additive range noise makes a range negative, that sample is omitted rather than clamped; this simple Gaussian model is limited near the sensor. There are no detection-range or field-of-view rules. The frontend converts valid radar observations to display coordinates, but the original range/bearing pair remains authoritative. Converted Cartesian errors can be correlated and vary with distance, so Cartesian sensor covariance must not be reused for radar.

Sensor outage windows are half-open `[start, end)` intervals in simulation seconds. Scheduled samples inside a window are omitted, including the start boundary and excluding the end boundary; true motion and the integer-indexed schedule continue. The simulator still evaluates the sensor at an outage slot and discards the result so seeded random draws for unaffected scheduled samples do not shift. Outage configuration is not passed to the tracker.

The prediction-only tracker uses state ordering `[x, y, vx, vy]` and a 4×4 covariance. It initializes position and position covariance from the first available Cartesian observation, sets velocity to zero, and uses the configured initial velocity standard deviation for velocity uncertainty. For elapsed time `dt`, `F(dt)` is the constant-velocity transition and continuous white acceleration noise uses spectral density `q` in m²/s³:

```text
Q(dt) = q * [[dt³/3, 0,      dt²/2, 0],
             [0,     dt³/3,  0,     dt²/2],
             [dt²/2, 0,      dt,    0],
             [0,     dt²/2,  0,     dt]]
```

Measurement updates use `z = [measured_x, measured_y]`, `H` selecting x/y, each observation’s Cartesian covariance as `R`, a linear solve for the innovation system, and the Joseph covariance form `(I-KH)P(I-KH)ᵀ + KRKᵀ`. Covariance is symmetrized after prediction and update. If an exact zero-noise case makes the innovation covariance singular, a least-squares solve is used only when the system is consistent; inconsistent singular measurements are rejected rather than silently regularized. Delayed observations where availability differs from measurement time are rejected. During an outage, prediction continues at output timestamps, measurement age grows, and the next available reading can correct the state. The model assumes approximately constant velocity; it cannot know about an unseen turn, and process noise represents uncertainty in that motion model.

The interactive Cartesian tracking layer draws truth, observations, estimates, estimated velocity, and the latest estimate's position uncertainty separately. It extracts the 2×2 position block from the 4×4 state covariance, diagonalizes the symmetric block, and scales the eigenvalues by the 2D 95% chi-square factor 5.991. Tiny negative eigenvalues attributable to floating-point roundoff are clamped to zero; materially invalid covariance is reported in the view. Zero and nearly-zero eigenvalues are valid degenerate ellipses. The ellipse is a model-based uncertainty illustration, not a guaranteed boundary.

Evaluation is a separate frontend path: at each reached output timestamp it compares the estimate with truth and with a baseline that holds the latest available measured position. It can exclude an initialization warm-up and reports the chosen period and timestamp count. Truth is never passed into the tracker, and metrics do not inspect future observations during playback. Radar remains sensor-only because the tracker measurement model currently accepts Cartesian x/y only.
