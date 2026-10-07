# Sensor Tracking Simulator

An educational, hardware-free simulator for learning motion and sensor measurements. It models one constant-velocity target with scheduled turns, two independently configurable Cartesian position sensors, and a simplified radar-style range/bearing mode. It does not implement targeting or engagement features.

## Prerequisites

- Python 3.12 and [uv](https://docs.astral.sh/uv/)
- Node.js 20+ and npm

## Run locally

In one terminal:

```bash
cd backend
uv sync
uv run uvicorn app.main:app --reload --port 8000
```

In another terminal:

```bash
cd frontend
npm install
npm run dev
```

Open the Vite URL shown in the terminal. The frontend expects the API at `http://localhost:8000` and supports `VITE_API_URL` for another URL.

## Checks

```bash
cd backend && uv run pytest
cd frontend && npm run typecheck && npm run build
```

See [docs/architecture.md](docs/architecture.md) and [docs/concepts.md](docs/concepts.md). Position observations are a simplified measurement model, not raw radar signals.

The legacy Cartesian sensor samples at integer multiples of its measurement interval, starting at t=0. The new position and camera sensors each have a stable ID, enabled flag, x/y noise standard deviations, x/y constant biases, interval, start offset, and outage windows. Each samples the same truth in world-frame meters at `start_offset + k × interval`, with its own reproducible seeded noise stream. Measurement noise controls are standard deviations in meters; covariance stores their squared values. With zero configured latency, availability equals measurement time; otherwise availability is delayed per sensor. The UI can also select a simplified radar-style sensor that reports range in meters and bearing in radians relative to the configured sensor heading. Bearing noise is entered in degrees and converted to radians; the covariance therefore mixes m² and rad².

The camera sensor is deliberately not an image simulator: it directly reports noisy world-frame x/y position. Real camera position estimates require scene geometry, depth reasoning, and calibration; optics, occlusion, and image processing are out of scope. Radar remains sensor-only. In Cartesian mode, Sensor A, Sensor B, and the fused tracker use the same generated observations for comparison; disabling one sensor only filters the observations sent to that comparison track.

Cartesian tracking uses one shared constant-velocity state. The fused run processes position and camera observations in timestamp order, predicts only when time advances, and applies each sensor's own covariance as a sequential measurement update. The API also returns comparable `sensor_a`, `sensor_b`, and `both` estimate runs that reuse the same generated observations. This assumes independent measurement errors and credible covariance values; it does not reject outliers or correct constant bias automatically.

Each sensor can add delivery latency. Measurement time remains the time the target was observed; availability time is measurement time plus latency, and values/covariance do not change. Playback and tracking reveal samples only at availability. The tracker uses a bounded educational replay coordinator: when a delayed sample is still within the configured history window, it restores a checkpoint before that measurement time, replays arrived observations chronologically, and updates only the current estimate. Previously emitted output timestamps are not rewritten. Older samples are rejected with a count and reason. Replay is intentionally readable rather than optimized and can repeat Kalman work as late samples arrive.

The sensor cards separate actual generated noise from the covariance reported to the tracker. Actual x/y standard deviations determine seeded Gaussian samples; reported standard deviations determine the diagonal covariance supplied to the Kalman update. Blank reported values match actual noise. Understating reported uncertainty makes a track overtrust that sensor. Constant bias is excluded from covariance and is not corrected automatically.

Constant x/y bias is added to every measurement but is intentionally excluded from covariance. Random noise can average down over repeated samples; a consistent bias does not.

## Learning experiments

Use the learning preset selector in the UI:

1. **Same motion, slower sensor updates**: compare the same truth path with fewer observation samples.
2. **Same motion, increased measurement noise**: keep the seed fixed and compare scatter without changing motion.
3. **Same bearing uncertainty, target farther from the radar-style sensor**: compare the same angular uncertainty at a greater range and observe the larger sideways position error.

Fusion comparison presets:

1. **Two independent sensors**: compare unbiased sensors with different noise and update rates.
2. **Sensor A outage**: let the camera continue while the position sensor is unavailable.
3. **Late camera**: compare identical camera measurements delivered after a fixed latency.
4. **Biased camera, low reported uncertainty**: see why averaging does not remove bias and why an overconfident covariance can make fusion worse.

The optional noise overlay illustrates configured standard deviations. It is not a guaranteed bound or tracker confidence region. Radar dots are converted to world coordinates for display, while the original range/bearing measurement remains authoritative.

Outage scenarios use half-open `[start, end)` windows. The drone continues its true motion and the sensor schedule resumes normally after the window. During the gap, the tracker predicts without measurements, preserves the last measurement timestamp, and exposes measurement age. It does not receive the outage schedule or any true turn information.

In Cartesian mode, the tracking view separates the true path, measured dots, causal estimated track, estimated velocity, and the latest 95% position uncertainty ellipse. The ellipse uses the x/y covariance block and the 2D chi-square factor 5.991; it reflects the filter's model assumptions and is not a guaranteed boundary. Tracker controls include acceleration-noise spectral density `q` in m²/s³ (motion-model uncertainty) and initial velocity uncertainty in m/s. Changing run controls requires **Run simulation**; playback controls only move through the completed result.

The tracker evaluation compares the causal estimate with a baseline that holds the latest available measured position at the same emitted output timestamps. It reports position RMSE after an explicitly selected warm-up period. Truth is used only by simulator visualization/evaluation code, never as tracker input. A low `q` favors smoother estimates; a high `q` lets the filter respond more readily to motion changes. A constant-velocity filter can lag behind turns, and a biased sensor can produce a confidently wrong estimate.

## Next milestones

The simulator supports straight flight, scheduled constant-speed turns, controlled per-sensor outages, two Cartesian observation streams, asynchronous shared-state fusion, bounded delayed-observation replay, and an interactive comparison view with truth, observations, three causal tracks, selectable uncertainty ellipses, and playback-safe RMSE. Radar samples at the exact scheduled truth time; undefined-origin and negative-range samples are omitted. Deliberately out of scope for later milestones: nonlinear radar tracking, correlated-error fusion, more advanced delayed-observation strategies, multi-target data association, Doppler, clutter, missed detections, field-of-view/detection-range rules, and track deletion/lifecycle.
