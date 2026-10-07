from app.schemas.models import SimulationConfig, TurnEvent


def example_scenarios() -> dict[str, SimulationConfig]:
    """Return editable starter configurations for the motion milestone."""
    return {
        "straight_flight": SimulationConfig(),
        "gradual_90_degree_turn": SimulationConfig(
            duration_seconds=20,
            turn_events=[
                TurnEvent(
                    start_time_seconds=8,
                    duration_seconds=4,
                    turn_rate_degrees_per_second=22.5,
                )
            ],
        ),
        "s_shaped_path": SimulationConfig(
            duration_seconds=24,
            turn_events=[
                TurnEvent(
                    start_time_seconds=6,
                    duration_seconds=3,
                    turn_rate_degrees_per_second=30,
                ),
                TurnEvent(
                    start_time_seconds=13,
                    duration_seconds=6,
                    turn_rate_degrees_per_second=-30,
                ),
            ],
        ),
    }
