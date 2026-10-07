import { stateAtTime } from './playback';
import type { SensorObservation, SimulationResult } from '../types/models';

export type ObservationEvaluation = { observation: SensorObservation; truthX: number; truthY: number; errorX: number; errorY: number; distance: number };

export function latestAvailableObservationEvaluation(result: SimulationResult, playbackTime: number): ObservationEvaluation | null {
  const latest = [...result.observations].reverse().find((observation) => observation.availability_timestamp <= playbackTime + 1e-9);
  if (!latest) return null;
  const truth = stateAtTime(result.truth_history, latest.measurement_timestamp);
  const errorX = latest.measurement_values[0] - truth.x;
  const errorY = latest.measurement_values[1] - truth.y;
  return {observation:latest, truthX:truth.x, truthY:truth.y, errorX, errorY, distance:Math.hypot(errorX,errorY)};
}
