# Concepts

- **Ground truth** is the actual simulated state, used for drawing and evaluation, never by a tracker.
- **Measurement noise** is random sensor error; x and y currently use independent Gaussian noise.
- **Measurement covariance** describes uncertainty; diagonal entries contain the x/y noise variance.
- **Sensor update rate** is the interval between measurements, independent of the integration timestep.
- **State estimation** infers position and velocity from imperfect observations. The Kalman filter is the next milestone and is intentionally not implemented yet.
- **Heading** is the direction of travel measured counterclockwise from positive x. Configuration uses degrees; motion math uses radians.
