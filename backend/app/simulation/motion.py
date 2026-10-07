import math

from app.schemas.models import TargetState, TurnEvent

TURN_EPSILON = 1e-10


def _validate_events(turn_events: list[TurnEvent]) -> list[TurnEvent]:
    events = sorted(turn_events, key=lambda event: event.start_time_seconds)
    for previous, current in zip(events, events[1:]):
        if current.start_time_seconds < previous.end_time_seconds:
            raise ValueError("turn events must not overlap")
    return events


def advance(state: TargetState, dt: float, turn_rate_degrees_per_second: float = 0.0) -> TargetState:
    """Advance analytically at a constant turn rate while preserving speed."""
    if dt < 0:
        raise ValueError("dt must be non-negative")

    speed = math.hypot(state.vx, state.vy)
    omega = math.radians(turn_rate_degrees_per_second)
    heading = math.atan2(state.vy, state.vx)
    if abs(omega) < TURN_EPSILON or speed == 0:
        x = state.x + state.vx * dt
        y = state.y + state.vy * dt
        vx, vy = state.vx, state.vy
    else:
        next_heading = heading + omega * dt
        x = state.x + speed / omega * (math.sin(next_heading) - math.sin(heading))
        y = state.y + speed / omega * (math.cos(heading) - math.cos(next_heading))
        vx, vy = speed * math.cos(next_heading), speed * math.sin(next_heading)
    return state.model_copy(update={"timestamp": round(state.timestamp + dt, 10), "x": x, "y": y, "vx": vx, "vy": vy})


def advance_with_turns(state: TargetState, dt: float, turn_events: list[TurnEvent]) -> TargetState:
    """Advance through a time interval, splitting exactly at turn boundaries."""
    if dt < 0:
        raise ValueError("dt must be non-negative")
    events = _validate_events(turn_events)
    current = state
    remaining = dt
    while remaining > TURN_EPSILON:
        current_time = current.timestamp
        active = next(
            (
                event
                for event in events
                if event.start_time_seconds <= current_time + TURN_EPSILON
                and event.end_time_seconds > current_time + TURN_EPSILON
            ),
            None,
        )
        boundary = current_time + remaining
        if active is not None:
            boundary = min(boundary, active.end_time_seconds)
            rate = active.turn_rate_degrees_per_second
        else:
            future_starts = [
                event.start_time_seconds
                for event in events
                if event.start_time_seconds > current_time + TURN_EPSILON
            ]
            if future_starts:
                boundary = min(boundary, min(future_starts))
            rate = 0.0
        segment = boundary - current_time
        if segment <= TURN_EPSILON:
            current = current.model_copy(update={"timestamp": round(boundary, 10)})
            continue
        current = advance(current, segment, rate)
        remaining -= segment
    return current
