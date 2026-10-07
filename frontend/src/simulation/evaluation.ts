import { stateAtTime } from './playback';
import { measurementToWorldCoordinates } from './measurement';
import type { SensorObservation, SimulationResult } from '../types/models';

export type ObservationEvaluation = { observation: SensorObservation; truthX: number; truthY: number; errorX: number; errorY: number; distance: number };
export type ObservationMetrics = { meanErrorX: number; meanErrorY: number; positionRmse: number; observationCount: number };
export type TrackingMetrics = { trackerPositionRmse: number; baselinePositionRmse: number; trackerObservationCount: number; evaluationStartSeconds: number; evaluationEndSeconds: number; warmupSeconds: number };
export type ComparisonTrackingMetrics = { trackerPositionRmse: Record<string, number>; commonStartSeconds: number; commonEndSeconds: number; commonTimestampCount: number; warmupSeconds: number };

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

export function trackingMetrics(result: SimulationResult, playbackTime: number, warmupSeconds: number): TrackingMetrics | null {
  if (result.configuration.sensor_type !== 'cartesian_position') return null;
  const estimates = result.estimates.filter((estimate) => estimate.timestamp <= playbackTime + 1e-9 && estimate.timestamp >= warmupSeconds);
  if (estimates.length === 0) return { trackerPositionRmse: 0, baselinePositionRmse: 0, trackerObservationCount: 0, evaluationStartSeconds: warmupSeconds, evaluationEndSeconds: 0, warmupSeconds };
  let trackerSquaredError = 0;
  let baselineSquaredError = 0;
  let included = 0;
  let evaluationEndSeconds = warmupSeconds;
  for (const estimate of estimates) {
    const availableObservations = result.observations.filter((observation) => observation.availability_timestamp <= estimate.timestamp + 1e-9);
    const latest = availableObservations[availableObservations.length - 1];
    if (!latest) continue;
    const truth = stateAtTime(result.truth_history, estimate.timestamp);
    const [baselineX, baselineY] = measurementToWorldCoordinates(latest, result.configuration);
    trackerSquaredError += (estimate.estimated_position[0] - truth.x) ** 2 + (estimate.estimated_position[1] - truth.y) ** 2;
    baselineSquaredError += (baselineX - truth.x) ** 2 + (baselineY - truth.y) ** 2;
    included += 1;
    evaluationEndSeconds = estimate.timestamp;
  }
  if (included === 0) return { trackerPositionRmse: 0, baselinePositionRmse: 0, trackerObservationCount: 0, evaluationStartSeconds: warmupSeconds, evaluationEndSeconds: 0, warmupSeconds };
  return { trackerPositionRmse: Math.sqrt(trackerSquaredError / included), baselinePositionRmse: Math.sqrt(baselineSquaredError / included), trackerObservationCount: included, evaluationStartSeconds: warmupSeconds, evaluationEndSeconds, warmupSeconds };
}

export function comparisonTrackingMetrics(result: SimulationResult, playbackTime: number, warmupSeconds: number): ComparisonTrackingMetrics | null {
  if (result.configuration.sensor_type !== 'cartesian_position') return null;
  const keys = ['sensor_a', 'sensor_b', 'both'];
  const reached = Object.fromEntries(keys.map((key) => [key, (result.comparison_estimates?.[key] ?? []).filter((estimate) => estimate.timestamp <= playbackTime + 1e-9 && estimate.timestamp >= warmupSeconds)])) as Record<string, SimulationResult['estimates']>;
  const timestampSets = keys.map((key) => new Set(reached[key].map((estimate) => estimate.timestamp.toFixed(9))));
  const common = [...timestampSets[0]].filter((timestamp) => timestampSets.every((set) => set.has(timestamp))).sort((a, b) => Number(a) - Number(b));
  const rmse: Record<string, number> = Object.fromEntries(keys.map((key) => [key, 0]));
  if (common.length === 0) return { trackerPositionRmse:rmse, commonStartSeconds:warmupSeconds, commonEndSeconds:0, commonTimestampCount:0, warmupSeconds };
  for (const timestamp of common) {
    const time = Number(timestamp); const truth = stateAtTime(result.truth_history, time);
    for (const key of keys) {
      const estimate = reached[key].find((candidate) => candidate.timestamp.toFixed(9) === timestamp);
      if (!estimate) continue;
      rmse[key] += (estimate.estimated_position[0] - truth.x) ** 2 + (estimate.estimated_position[1] - truth.y) ** 2;
    }
  }
  for (const key of keys) rmse[key] = Math.sqrt(rmse[key] / common.length);
  return { trackerPositionRmse:rmse, commonStartSeconds:Number(common[0]), commonEndSeconds:Number(common[common.length - 1]), commonTimestampCount:common.length, warmupSeconds };
}
