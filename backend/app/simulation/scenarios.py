from app.schemas.models import SensorOutage, SimulationConfig, TurnEvent


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
        "straight_with_outage": SimulationConfig(
            duration_seconds=20,
            outage_windows=[SensorOutage(start_time_seconds=5, end_time_seconds=8)],
        ),
        "turn_with_measurements": SimulationConfig(
            duration_seconds=20,
            turn_events=[TurnEvent(start_time_seconds=8, duration_seconds=4, turn_rate_degrees_per_second=22.5)],
        ),
        "turn_during_outage": SimulationConfig(
            duration_seconds=20,
            turn_events=[TurnEvent(start_time_seconds=8, duration_seconds=4, turn_rate_degrees_per_second=22.5)],
            outage_windows=[SensorOutage(start_time_seconds=7, end_time_seconds=13)],
        ),
    }
