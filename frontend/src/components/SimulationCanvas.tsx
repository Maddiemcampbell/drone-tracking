import { useEffect, useRef } from 'react';
import { latestAvailableObservation, measurementToWorldCoordinates, radarWorldAngleRadians } from '../simulation/measurement';
import { positionEllipsePoints } from '../simulation/uncertainty';
import { stateAtTime } from '../simulation/playback';
import type { SensorObservation, TargetState, TrackEstimate, SimulationResult } from '../types/models';

const VELOCITY_ARROW_SECONDS = 0.2;
const EPSILON = 1e-9;

function drawArrow(ctx: CanvasRenderingContext2D, fromX: number, fromY: number, toX: number, toY: number, color: string, width = 2.5) {
  const angle = Math.atan2(toY - fromY, toX - fromX);
  const head = 9;
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
  ctx.beginPath(); ctx.moveTo(fromX, fromY); ctx.lineTo(toX, toY); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(toX, toY); ctx.lineTo(toX - head * Math.cos(angle - Math.PI / 6), toY - head * Math.sin(angle - Math.PI / 6)); ctx.lineTo(toX - head * Math.cos(angle + Math.PI / 6), toY - head * Math.sin(angle + Math.PI / 6)); ctx.closePath(); ctx.fill();
}

function drawWorldRay(ctx: CanvasRenderingContext2D, toCanvas: (x: number, y: number) => [number, number], origin: [number, number], angle: number, length: number) {
  const from = toCanvas(origin[0], origin[1]);
  const to = toCanvas(origin[0] + length * Math.cos(angle), origin[1] + length * Math.sin(angle));
  ctx.strokeStyle = '#f7d774'; ctx.lineWidth = 1.5; ctx.setLineDash([5, 5]);
  ctx.beginPath(); ctx.moveTo(from[0], from[1]); ctx.lineTo(to[0], to[1]); ctx.stroke(); ctx.setLineDash([]);
}

function drawNoiseOverlay(ctx: CanvasRenderingContext2D, observation: SensorObservation, config: SimulationResult['configuration'], toCanvas: (x: number, y: number) => [number, number], scale: number) {
  const [measuredX, measuredY] = measurementToWorldCoordinates(observation, config);
  const measuredPoint = toCanvas(measuredX, measuredY);
  ctx.strokeStyle = '#f7d774'; ctx.lineWidth = 1.5; ctx.setLineDash([4, 4]);
  if (observation.measurement_type === 'cartesian_position') {
    const halfWidth = config.measurement_noise_std_x * scale;
    const halfHeight = config.measurement_noise_std_y * scale;
    if (halfWidth > EPSILON || halfHeight > EPSILON) {
      ctx.beginPath(); ctx.ellipse(measuredPoint[0], measuredPoint[1], Math.max(halfWidth, 1), Math.max(halfHeight, 1), 0, 0, Math.PI * 2); ctx.stroke();
    }
  } else {
    const origin: [number, number] = [config.sensor_position_x, config.sensor_position_y];
    const [range] = observation.measurement_values;
    const worldAngle = radarWorldAngleRadians(observation, config);
    const rangeSigma = config.range_noise_std_meters;
    if (rangeSigma > EPSILON) {
      const sensorPoint = toCanvas(origin[0], origin[1]);
      [Math.max(0, range - rangeSigma), range + rangeSigma].forEach((radius) => { ctx.beginPath(); ctx.arc(sensorPoint[0], sensorPoint[1], radius * scale, 0, Math.PI * 2); ctx.stroke(); });
    }
    const bearingSigma = config.bearing_noise_std_degrees * Math.PI / 180;
    if (bearingSigma > EPSILON && range > EPSILON) {
      drawWorldRay(ctx, toCanvas, origin, worldAngle - bearingSigma, range);
      drawWorldRay(ctx, toCanvas, origin, worldAngle + bearingSigma, range);
    }
  }
  ctx.setLineDash([]);
}

function drawCoordinatePath(ctx: CanvasRenderingContext2D, points: [number, number][], toCanvas: (x: number, y: number) => [number, number], color: string, dashed = false) {
  if (points.length === 0) return;
  ctx.strokeStyle = color; ctx.lineWidth = dashed ? 2 : 3; ctx.setLineDash(dashed ? [7, 5] : []); ctx.beginPath();
  points.forEach((point, index) => { const [x, y] = toCanvas(point[0], point[1]); if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
  ctx.stroke(); ctx.setLineDash([]);
}

function drawEstimateMarker(ctx: CanvasRenderingContext2D, estimate: TrackEstimate, toCanvas: (x: number, y: number) => [number, number], color: string) {
  const [x, y] = toCanvas(estimate.estimated_position[0], estimate.estimated_position[1]);
  ctx.fillStyle = color; ctx.strokeStyle = '#f6f7fb'; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 8, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  const [arrowX, arrowY] = toCanvas(estimate.estimated_position[0] + estimate.estimated_velocity[0] * VELOCITY_ARROW_SECONDS, estimate.estimated_position[1] + estimate.estimated_velocity[1] * VELOCITY_ARROW_SECONDS);
  if (Math.hypot(estimate.estimated_velocity[0], estimate.estimated_velocity[1]) > EPSILON) drawArrow(ctx, x, y, arrowX, arrowY, '#ff86d5', 2.5);
}

export function SimulationCanvas({ result, time, showTruth, showObservations, showTrack, showUncertainty, showNoiseOverlay }: { result: SimulationResult | null; time: number; showTruth: boolean; showObservations: boolean; showTrack: boolean; showUncertainty: boolean; showNoiseOverlay: boolean }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const canvas = ref.current, ctx = canvas?.getContext('2d'); if (!canvas || !ctx) return;
    const width = canvas.width, height = canvas.height; ctx.clearRect(0, 0, width, height); ctx.fillStyle = '#0d1726'; ctx.fillRect(0, 0, width, height);
    if (!result || result.truth_history.length === 0) { ctx.fillStyle = '#aab8c8'; ctx.font = '18px system-ui, sans-serif'; ctx.fillText('Run a scenario to explore its motion.', 32, height / 2); return; }
    const { configuration: config, truth_history: truth, estimates } = result;
    const observedWorld = result.observations.map((observation) => measurementToWorldCoordinates(observation, config));
    const estimateWorld = estimates.map((estimate) => [estimate.estimated_position[0], estimate.estimated_position[1]] as [number, number]);
    const ellipseWorld: [number, number][] = [];
    let covarianceWarning = '';
    for (const estimate of estimates) {
      try { ellipseWorld.push(...positionEllipsePoints(estimate.estimated_position[0], estimate.estimated_position[1], estimate.state_covariance)); }
      catch (error) { covarianceWarning = error instanceof Error ? error.message : 'position covariance is invalid'; }
    }
    const worldPoints: [number, number][] = [...truth.map((state) => [state.x, state.y] as [number, number]), ...observedWorld, ...estimateWorld, ...ellipseWorld, [config.sensor_position_x, config.sensor_position_y]];
    const xs = worldPoints.map((point) => point[0]), ys = worldPoints.map((point) => point[1]);
    const rawMinX = Math.min(...xs), rawMaxX = Math.max(...xs), rawMinY = Math.min(...ys), rawMaxY = Math.max(...ys), rawSpan = Math.max(rawMaxX - rawMinX, rawMaxY - rawMinY, 1), margin = Math.max(rawSpan * .08, 1), minX = rawMinX - margin, maxX = rawMaxX + margin, minY = rawMinY - margin, maxY = rawMaxY + margin, padding = 54, scale = Math.min((width - padding * 2) / (maxX - minX), (height - padding * 2) / (maxY - minY)), centerX = (minX + maxX) / 2, centerY = (minY + maxY) / 2;
    const toCanvas = (x: number, y: number): [number, number] => [width / 2 + (x - centerX) * scale, height / 2 - (y - centerY) * scale];
    const axisX = toCanvas(0, 0)[0], axisY = toCanvas(0, 0)[1];
    const drawAxis = (horizontal: boolean) => { const inBounds = horizontal ? axisY >= padding && axisY <= height - padding : axisX >= padding && axisX <= width - padding; const startX = horizontal ? padding : (inBounds ? axisX : padding), startY = horizontal ? (inBounds ? axisY : height - padding) : height - padding, endX = horizontal ? width - padding : startX, endY = horizontal ? startY : padding; ctx.strokeStyle = '#31445b'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(startX, startY); ctx.lineTo(endX, endY); ctx.stroke(); drawArrow(ctx, horizontal ? endX - 16 : startX, horizontal ? endY : endY + 16, endX, endY, '#71869d', 1.5); ctx.fillStyle = '#8ea1b5'; ctx.font = '13px system-ui, sans-serif'; ctx.fillText(horizontal ? 'x (m) →' : 'y (m) ↑', horizontal ? endX - 48 : startX + 8, horizontal ? endY - 8 : endY + 16); };
    drawAxis(true); drawAxis(false);
    const current = stateAtTime(truth, Math.min(Math.max(time, 0), config.duration_seconds));
    const visibleTruth = truth.filter((state) => state.timestamp <= current.timestamp + EPSILON), truthPath = visibleTruth.length > 0 ? visibleTruth.map((state) => [state.x, state.y] as [number, number]) : [[truth[0].x, truth[0].y] as [number, number]]; if (truthPath.length > 0 && visibleTruth[visibleTruth.length - 1]?.timestamp < current.timestamp - EPSILON) truthPath.push([current.x, current.y]);
    if (showTruth) drawCoordinatePath(ctx, truthPath, toCanvas, '#49dcb1');
    const latest = latestAvailableObservation(result, current.timestamp);
    if (showObservations) {
      ctx.fillStyle = '#ffb454'; result.observations.filter((observation) => observation.availability_timestamp <= current.timestamp + EPSILON).forEach((observation) => { const [x, y] = toCanvas(...measurementToWorldCoordinates(observation, config)); ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fill(); });
    }
    if (config.sensor_type === 'range_bearing') {
      const sensorPoint = toCanvas(config.sensor_position_x, config.sensor_position_y); ctx.fillStyle = '#f7d774'; ctx.strokeStyle = '#f6f7fb'; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(sensorPoint[0], sensorPoint[1], 7, 0, Math.PI * 2); ctx.fill();
      const heading = config.sensor_heading_degrees * Math.PI / 180; drawArrow(ctx, sensorPoint[0], sensorPoint[1], ...toCanvas(config.sensor_position_x + 4 * Math.cos(heading), config.sensor_position_y + 4 * Math.sin(heading)), '#f7d774', 2);
      if (showObservations && latest) { const measuredPoint = toCanvas(...measurementToWorldCoordinates(latest, config)); ctx.strokeStyle = '#ffb454'; ctx.lineWidth = 1.5; ctx.setLineDash([6, 4]); ctx.beginPath(); ctx.moveTo(sensorPoint[0], sensorPoint[1]); ctx.lineTo(measuredPoint[0], measuredPoint[1]); ctx.stroke(); ctx.setLineDash([]); }
      if (showObservations && showNoiseOverlay && latest) drawNoiseOverlay(ctx, latest, config, toCanvas, scale);
    } else if (showObservations && showNoiseOverlay && latest) drawNoiseOverlay(ctx, latest, config, toCanvas, scale);
    const visibleEstimates = estimates.filter((estimate) => estimate.timestamp <= current.timestamp + EPSILON);
    const latestEstimate = visibleEstimates[visibleEstimates.length - 1];
    if (showTrack && visibleEstimates.length > 0) {
      drawCoordinatePath(ctx, visibleEstimates.map((estimate) => [estimate.estimated_position[0], estimate.estimated_position[1]]), toCanvas, '#9b8cff', true);
      if (showUncertainty && latestEstimate) {
        try { const points = positionEllipsePoints(latestEstimate.estimated_position[0], latestEstimate.estimated_position[1], latestEstimate.state_covariance); if (points.length > 0) { ctx.strokeStyle = '#d0c7ff'; ctx.fillStyle = '#9b8cff22'; ctx.lineWidth = 2; ctx.setLineDash([5, 4]); ctx.beginPath(); points.forEach((point, index) => { const [x, y] = toCanvas(point[0], point[1]); if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); }); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.setLineDash([]); } }
        catch (error) { covarianceWarning = error instanceof Error ? error.message : 'position covariance is invalid'; }
      }
      drawEstimateMarker(ctx, latestEstimate, toCanvas, '#9b8cff');
    }
    const [currentX, currentY] = toCanvas(current.x, current.y), [truthArrowX, truthArrowY] = toCanvas(current.x + current.vx * VELOCITY_ARROW_SECONDS, current.y + current.vy * VELOCITY_ARROW_SECONDS); if (Math.hypot(current.vx, current.vy) > EPSILON) drawArrow(ctx, currentX, currentY, truthArrowX, truthArrowY, '#f6f7fb'); if (showTruth) { ctx.fillStyle = '#49dcb1'; ctx.strokeStyle = '#f6f7fb'; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(currentX, currentY, 9, 0, Math.PI * 2); ctx.fill(); ctx.stroke(); }
    if (covarianceWarning) { ctx.fillStyle = '#ff9d9d'; ctx.font = '12px system-ui, sans-serif'; ctx.fillText(`Covariance warning: ${covarianceWarning}`, padding, height - 16); }
  }, [result, time, showTruth, showObservations, showTrack, showUncertainty, showNoiseOverlay]);
  return <canvas ref={ref} width={900} height={560} aria-label="Motion simulation world showing truth, observations, estimated track, velocity, and uncertainty" />;
}

export { VELOCITY_ARROW_SECONDS };
