import type { SensorObservation, SimulationConfig } from '../types/models';

const EPSILON = 1e-9;

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

export function latestAvailableObservation(result: { observations: SensorObservation[] }, playbackTime: number): SensorObservation | null {
  const available = result.observations.filter((observation) => observation.availability_timestamp <= playbackTime + EPSILON);
  return available.length > 0 ? available[available.length - 1] : null;
}

export function radarWorldAngleRadians(observation: SensorObservation, config: SimulationConfig): number {
  return config.sensor_heading_degrees * Math.PI / 180 + observation.measurement_values[1];
}
