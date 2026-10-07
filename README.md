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

## Next milestones

The simulator supports straight flight and scheduled constant-speed turns, including straight, gradual 90-degree, and S-shaped example scenarios. The observation-only constant-velocity Kalman filter is implemented and visualized as a dashed estimate path. Next: add sensor dropouts, range/bearing measurements, delayed measurements, and sensor fusion.
