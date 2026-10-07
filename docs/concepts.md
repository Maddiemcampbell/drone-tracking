# Concepts

- **Ground truth** is the actual simulated state, used for drawing and evaluation, never by a tracker.
- **Measurement noise** is random sensor error; x and y currently use independent Gaussian noise.
- **Measurement covariance** describes uncertainty; diagonal entries contain the x/y noise variance.
- **Sensor update rate** is the interval between measurements, independent of the integration timestep.
- **State estimation** infers position and velocity from imperfect observations. The Kalman filter is the next milestone and is intentionally not implemented yet.
- **Heading** is the direction of travel measured counterclockwise from positive x. Configuration uses degrees; motion math uses radians.
- **Standard deviation vs. variance**: σ describes the typical measurement error in meters; covariance stores σ² in its diagonal. x and y noise are independent.
- **Sampling schedule**: measurements occur at `k × interval` for integer k, including t=0 and any schedule time through the run duration. A sample is not snapped to the nearest simulation tick.
- **Radar-style measurement**: the simplified range/bearing sensor measures distance in meters and bearing relative to the sensor heading. Bearing is configured in degrees but stored in radians; its covariance entry is therefore in rad², while range covariance is in m².
- **Radar edge cases**: a target at the sensor origin has no defined bearing, so that observation is omitted. A negative noisy range is invalid and omitted rather than clamped. No field-of-view or detection-range logic is included.
- **Radar visualization**: the UI converts a valid range/bearing pair back to world x/y for display. That conversion is not a new Cartesian observation; the range/bearing pair is authoritative. Cartesian errors derived from it can be correlated and distance-dependent.
