import type { SensorObservation, SimulationConfig } from '../types/models';

export function measurementToWorldCoordinates(observation: SensorObservation, config: SimulationConfig): [number, number] {
  if (observation.measurement_type === 'cartesian_position') {
    return [observation.measurement_values[0], observation.measurement_values[1]];
  }
  const [range, bearing] = observation.measurement_values;
  const worldAngle = config.sensor_heading_degrees * Math.PI / 180 + bearing;
  return [
    config.sensor_position_x + range * Math.cos(worldAngle),
    config.sensor_position_y + range * Math.sin(worldAngle),
  ];
}
