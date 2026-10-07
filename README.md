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

## Next milestones

The simulator supports straight flight, scheduled constant-speed turns, and controlled sensor outages, including straight-with-outage, turn-with-measurements, and turn-during-outage scenarios. Radar samples at the exact scheduled truth time; undefined-origin and negative-range samples are omitted. The current tracker milestone is backend-only and Cartesian-only: it initializes from the first observation, predicts and applies causal measurement updates at exact timestamps, and emits estimates at simulation output timestamps. The frontend does not present these estimates as a finished tracking UI yet. Deliberately out of scope for later milestones: Doppler, clutter, missed detections, latency, field-of-view/detection-range rules, delayed-measurement handling, radar tracking, track deletion/lifecycle, and multi-sensor fusion.
