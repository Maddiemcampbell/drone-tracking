import { stateAtTime } from './playback';
import { measurementToWorldCoordinates } from './measurement';
import type { SensorObservation, SimulationResult } from '../types/models';

export type ObservationEvaluation = { observation: SensorObservation; truthX: number; truthY: number; errorX: number; errorY: number; distance: number };
export type ObservationMetrics = { meanErrorX: number; meanErrorY: number; positionRmse: number; observationCount: number };

export function availableObservationEvaluations(result: SimulationResult, playbackTime: number): ObservationEvaluation[] {
  return result.observations
    .filter((observation) => observation.availability_timestamp <= playbackTime + 1e-9)
    .map((observation) => {
      const truth = stateAtTime(result.truth_history, observation.measurement_timestamp);
      const [measurementX, measurementY] = measurementToWorldCoordinates(observation, result.configuration);
      const errorX = measurementX - truth.x;
      const errorY = measurementY - truth.y;
      return {observation, truthX:truth.x, truthY:truth.y, errorX, errorY, distance:Math.hypot(errorX,errorY)};
    });
}

export function observationMetrics(result: SimulationResult, playbackTime: number): ObservationMetrics {
  const evaluations = availableObservationEvaluations(result, playbackTime);
  if (evaluations.length === 0) return {meanErrorX:0, meanErrorY:0, positionRmse:0, observationCount:0};
  const sumX = evaluations.reduce((sum, evaluation) => sum + evaluation.errorX, 0);
  const sumY = evaluations.reduce((sum, evaluation) => sum + evaluation.errorY, 0);
  const meanSquaredPositionError = evaluations.reduce((sum, evaluation) => sum + evaluation.errorX ** 2 + evaluation.errorY ** 2, 0) / evaluations.length;
  return {meanErrorX:sumX / evaluations.length, meanErrorY:sumY / evaluations.length, positionRmse:Math.sqrt(meanSquaredPositionError), observationCount:evaluations.length};
}

export function latestAvailableObservationEvaluation(result: SimulationResult, playbackTime: number): ObservationEvaluation | null {
  const evaluations = availableObservationEvaluations(result, playbackTime);
  return evaluations.length > 0 ? evaluations[evaluations.length - 1] : null;
}
