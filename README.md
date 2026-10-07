# Sensor Tracking Simulator

An educational, hardware-free simulator for learning motion and sensor measurements. It models one constant-velocity target with scheduled turns and one active sensor: Cartesian position or simplified radar-style range/bearing. It does not implement targeting or engagement features.

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

The Cartesian sensor samples at integer multiples of its measurement interval, starting at t=0. Measurement noise controls are standard deviations in meters; the returned covariance stores their squared values. Sensor availability currently equals measurement time. The UI can also select a simplified radar-style sensor that reports range in meters and bearing in radians relative to the configured sensor heading. Bearing noise is entered in degrees and converted to radians; the covariance therefore mixes m² and rad².

Constant x/y bias is added to every measurement but is intentionally excluded from covariance. Random noise can average down over repeated samples; a consistent bias does not.

## Learning experiments

Use the learning preset selector in the UI:

1. **Same motion, slower sensor updates**: compare the same truth path with fewer observation samples.
2. **Same motion, increased measurement noise**: keep the seed fixed and compare scatter without changing motion.
3. **Same bearing uncertainty, target farther from the radar-style sensor**: compare the same angular uncertainty at a greater range and observe the larger sideways position error.

The optional noise overlay illustrates configured standard deviations. It is not a guaranteed bound or tracker confidence region. Radar dots are converted to world coordinates for display, while the original range/bearing measurement remains authoritative.

Outage scenarios use half-open `[start, end)` windows. The drone continues its true motion and the sensor schedule resumes normally after the window. During the gap, the tracker predicts without measurements, preserves the last measurement timestamp, and exposes measurement age. It does not receive the outage schedule or any true turn information.

In Cartesian mode, the tracking view separates the true path, measured dots, causal estimated track, estimated velocity, and the latest 95% position uncertainty ellipse. The ellipse uses the x/y covariance block and the 2D chi-square factor 5.991; it reflects the filter's model assumptions and is not a guaranteed boundary. Tracker controls include acceleration-noise spectral density `q` in m²/s³ (motion-model uncertainty) and initial velocity uncertainty in m/s. Changing run controls requires **Run simulation**; playback controls only move through the completed result.

The tracker evaluation compares the causal estimate with a baseline that holds the latest available measured position at the same emitted output timestamps. It reports position RMSE after an explicitly selected warm-up period. Truth is used only by simulator visualization/evaluation code, never as tracker input. A low `q` favors smoother estimates; a high `q` lets the filter respond more readily to motion changes. A constant-velocity filter can lag behind turns, and a biased sensor can produce a confidently wrong estimate.

## Next milestones

The simulator supports straight flight, scheduled constant-speed turns, controlled sensor outages, and an interactive Cartesian tracking view. Radar samples at the exact scheduled truth time; undefined-origin and negative-range samples are omitted. Deliberately out of scope for later milestones: nonlinear radar tracking, delayed observations, sensor fusion, multi-target data association, Doppler, clutter, missed detections, latency, field-of-view/detection-range rules, and track deletion/lifecycle.
