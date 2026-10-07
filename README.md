# Sensor Tracking Simulator

An educational, hardware-free simulator for learning radar measurements, state estimation, Kalman filtering, and sensor fusion. The first slice models one constant-velocity target and noisy Cartesian position observations. It does not implement targeting or engagement features.

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

## Motion experiments

1. Compare Straight and 90-degree turn at the same speed. Keep truth visible and scrub through the turn.
2. Increase measurement noise while keeping the random seed fixed. Compare the noisy dots without changing the true path.
3. Increase the sensor measurement interval and inspect the S-turn. The truth remains continuous while observations become less frequent.

## Next milestones

The simulator supports straight flight and scheduled constant-speed turns, including straight, gradual 90-degree, and S-shaped example scenarios, plus Cartesian and simplified radar-style observations. Radar samples at the exact scheduled truth time; undefined-origin and negative-range samples are omitted. Next: add sensor dropouts, delayed measurements, and sensor fusion.
