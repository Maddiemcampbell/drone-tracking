import { useEffect, useState } from 'react';
import type { ChangeEvent } from 'react';
import { SimulationCanvas } from './components/SimulationCanvas';
import { latestAvailableObservation, measurementToWorldCoordinates } from './simulation/measurement';
import { latestAvailableObservationEvaluation, observationMetrics, trackingMetrics } from './simulation/evaluation';
import { stateAtTime } from './simulation/playback';
import { simulate } from './simulation/api';
import type { SensorOutage, SimulationConfig, SimulationResult } from './types/models';
import './styles.css';

type Scenario = { label: string; description: string; config: SimulationConfig };
type LearningPreset = { label: string; description: string };

const baseConfig = { measurement_noise_std_x: 5, measurement_noise_std_y: 5, measurement_bias_x: 0, measurement_bias_y: 0, sensor_type: 'cartesian_position' as const, sensor_heading_degrees: 0, range_noise_std_meters: 5, bearing_noise_std_degrees: 2, tracker_initial_velocity_std_mps: 10, tracker_acceleration_noise_spectral_density: 1, outage_windows: [] as SensorOutage[], sensor_id: 'position-sensor-1', sensor_position_x: 0, sensor_position_y: 0 };
const scenarios: Record<string, Scenario> = {
  straight_flight: { label: 'Straight', description: 'Constant speed with no scheduled turns.', config: { ...baseConfig, duration_seconds: 20, simulation_timestep_seconds: .1, sensor_interval_seconds: .5, random_seed: 7, initial_x: 0, initial_y: 0, initial_speed: 10, initial_heading_degrees: 0, turn_events: [] } },
  gradual_90_degree_turn: { label: '90-degree turn', description: 'A gradual left turn that changes heading by 90 degrees.', config: { ...baseConfig, duration_seconds: 20, simulation_timestep_seconds: .1, sensor_interval_seconds: .5, random_seed: 7, initial_x: 0, initial_y: 0, initial_speed: 10, initial_heading_degrees: 0, turn_events: [{ start_time_seconds: 8, duration_seconds: 4, turn_rate_degrees_per_second: 22.5 }] } },
  s_shaped_path: { label: 'S-turn', description: 'Two opposite turns create an S-shaped path.', config: { ...baseConfig, duration_seconds: 24, simulation_timestep_seconds: .1, sensor_interval_seconds: .5, random_seed: 7, initial_x: 0, initial_y: 0, initial_speed: 10, initial_heading_degrees: 0, turn_events: [{ start_time_seconds: 6, duration_seconds: 3, turn_rate_degrees_per_second: 30 }, { start_time_seconds: 13, duration_seconds: 6, turn_rate_degrees_per_second: -30 }] } },
  straight_with_outage: { label: 'Straight + outage', description: 'Straight flight with a temporary sensor outage from 5 to 8 seconds.', config: { ...baseConfig, duration_seconds: 20, simulation_timestep_seconds: .1, sensor_interval_seconds: .5, random_seed: 7, initial_x: 0, initial_y: 0, initial_speed: 10, initial_heading_degrees: 0, turn_events: [], outage_windows: [{ start_time_seconds: 5, end_time_seconds: 8 }] } },
  turn_with_measurements: { label: 'Turn + measurements', description: 'A gradual turn while measurements remain available.', config: { ...baseConfig, duration_seconds: 20, simulation_timestep_seconds: .1, sensor_interval_seconds: .5, random_seed: 7, initial_x: 0, initial_y: 0, initial_speed: 10, initial_heading_degrees: 0, turn_events: [{ start_time_seconds: 8, duration_seconds: 4, turn_rate_degrees_per_second: 22.5 }] } },
  turn_during_outage: { label: 'Turn during outage', description: 'The drone turns while measurements are unavailable from 7 to 13 seconds.', config: { ...baseConfig, duration_seconds: 20, simulation_timestep_seconds: .1, sensor_interval_seconds: .5, random_seed: 7, initial_x: 0, initial_y: 0, initial_speed: 10, initial_heading_degrees: 0, turn_events: [{ start_time_seconds: 8, duration_seconds: 4, turn_rate_degrees_per_second: 22.5 }], outage_windows: [{ start_time_seconds: 7, end_time_seconds: 13 }] } },
};
const learningPresets: Record<string, LearningPreset> = {
  slower_updates: { label: 'Same motion · slower updates', description: 'Keeps the trajectory and noise model, but samples less often.' },
  more_noise: { label: 'Same motion · more measurement noise', description: 'Keeps the trajectory and update rate, but makes measurements less accurate.' },
  farther_radar: { label: 'Same bearing uncertainty · farther radar target', description: 'Switches to radar, keeps 2° bearing noise, and moves the sensor 40 m behind the start.' },
};
const trackingPresets: Record<string, LearningPreset> = {
  straight_noisy: { label: 'Straight flight · noisy observations', description: 'Cartesian tracking with repeatable measurement noise.' },
  turn_low_process_noise: { label: 'Turn · low process noise', description: 'A smoother model that may lag behind the turn.' },
  turn_high_process_noise: { label: 'Turn · high process noise', description: 'A more responsive model that allows more motion variation.' },
  straight_outage: { label: 'Straight flight · outage', description: 'Prediction continues while scheduled measurements are missing.' },
  turn_outage: { label: 'Turn during outage', description: 'The constant-velocity tracker cannot see the unseen turn.' },
};

function copyConfig(config: SimulationConfig): SimulationConfig { return { ...config, turn_events: config.turn_events.map((event) => ({ ...event })), outage_windows: config.outage_windows.map((window) => ({ ...window })) }; }
function normalizeDegrees(degrees: number): number { return ((degrees + 180) % 360 + 360) % 360 - 180; }
function formatHeading(state: { vx: number; vy: number }, fallback: number): string { if (Math.hypot(state.vx, state.vy) < 1e-9) return `${fallback.toFixed(1)}° (stationary)`; return `${((Math.atan2(state.vy, state.vx) * 180 / Math.PI + 360) % 360).toFixed(1)}°`; }

export default function App() {
  const [scenarioId, setScenarioId] = useState('straight_flight');
  const [config, setConfig] = useState(() => copyConfig(scenarios.straight_flight.config));
  const [result, setResult] = useState<SimulationResult | null>(null);
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [showTruth, setShowTruth] = useState(true);
  const [showObservations, setShowObservations] = useState(true);
  const [showTrack, setShowTrack] = useState(true);
  const [showUncertainty, setShowUncertainty] = useState(true);
  const [showNoiseOverlay, setShowNoiseOverlay] = useState(false);
  const [warmupSeconds, setWarmupSeconds] = useState(2);
  const [error, setError] = useState('');
  const currentScenario = scenarios[scenarioId];
  const visualConfig = result?.configuration ?? config;
  const currentState = result ? stateAtTime(result.truth_history, time) : null;
  const latestObservation = result ? latestAvailableObservation(result, time) : null;
  const latestEstimate = result ? [...result.estimates].reverse().find((estimate) => estimate.timestamp <= time + 1e-9) ?? null : null;
  const evaluation = result ? latestAvailableObservationEvaluation(result, time) : null;
  const metrics = result ? observationMetrics(result, time) : null;
  const trackerMetrics = result ? trackingMetrics(result, time, warmupSeconds) : null;
  const update = (key: keyof SimulationConfig) => (event: ChangeEvent<HTMLInputElement>) => setConfig({ ...config, [key]: Number(event.target.value) });

  useEffect(() => {
    if (!playing || !result) return undefined;
    const interval = window.setInterval(() => setTime((current) => {
      const next = current + .05;
      if (next >= result.configuration.duration_seconds) { setPlaying(false); return result.configuration.duration_seconds; }
      return next;
    }), 50);
    return () => window.clearInterval(interval);
  }, [playing, result]);

  async function runSimulation() {
    setError(''); setPlaying(false);
    try { setResult(await simulate(config)); setTime(0); }
    catch (cause) { setError(cause instanceof Error ? cause.message : 'Unable to run simulation'); }
  }

  function chooseScenario(event: ChangeEvent<HTMLSelectElement>) {
    const nextId = event.target.value;
    setScenarioId(nextId); setConfig(copyConfig(scenarios[nextId].config)); setResult(null); setTime(0); setPlaying(false); setError('');
  }

  function applyLearningPreset(event: ChangeEvent<HTMLSelectElement>) {
    const presetId = event.target.value;
    if (!presetId) return;
    const next = { ...config };
    if (presetId === 'slower_updates') next.sensor_interval_seconds = 1.5;
    if (presetId === 'more_noise') {
      if (next.sensor_type === 'cartesian_position') { next.measurement_noise_std_x = 12; next.measurement_noise_std_y = 12; }
      else { next.range_noise_std_meters = 12; next.bearing_noise_std_degrees = 5; }
    }
    if (presetId === 'farther_radar') { next.sensor_type = 'range_bearing'; next.sensor_position_x = -40; next.sensor_position_y = 0; next.sensor_heading_degrees = 0; next.bearing_noise_std_degrees = 2; next.range_noise_std_meters = 5; next.measurement_bias_x = 0; next.measurement_bias_y = 0; }
    setConfig(next); setResult(null); setTime(0); setPlaying(false); setError('');
  }

  function applyTrackingPreset(event: ChangeEvent<HTMLSelectElement>) {
    const presetId = event.target.value;
    if (!presetId) return;
    const scenarioByPreset: Record<string, string> = { straight_noisy: 'straight_flight', turn_low_process_noise: 'gradual_90_degree_turn', turn_high_process_noise: 'gradual_90_degree_turn', straight_outage: 'straight_with_outage', turn_outage: 'turn_during_outage' };
    const next = copyConfig(scenarios[scenarioByPreset[presetId]].config);
    if (presetId === 'straight_noisy') { next.measurement_noise_std_x = 5; next.measurement_noise_std_y = 5; next.tracker_acceleration_noise_spectral_density = 1; }
    if (presetId === 'turn_low_process_noise') next.tracker_acceleration_noise_spectral_density = .1;
    if (presetId === 'turn_high_process_noise') next.tracker_acceleration_noise_spectral_density = 10;
    setScenarioId(scenarioByPreset[presetId]); setConfig(next); setResult(null); setTime(0); setPlaying(false); setError('');
  }

  function updateOutage(index: number, key: keyof SensorOutage, event: ChangeEvent<HTMLInputElement>) {
    const outage_windows = config.outage_windows.map((window, windowIndex) => windowIndex === index ? { ...window, [key]: Number(event.target.value) } : window);
    setConfig({ ...config, outage_windows });
  }

  function addOutage() {
    setConfig({ ...config, outage_windows: [...config.outage_windows, { start_time_seconds: 5, end_time_seconds: 8 }] });
  }

  function removeOutage(index: number) {
    setConfig({ ...config, outage_windows: config.outage_windows.filter((_, windowIndex) => windowIndex !== index) });
  }

  function resetPlayback() { setPlaying(false); setTime(0); }

  return <main>
    <header className="hero"><p className="eyebrow">Motion lab · batch simulation</p><h1>Truth, sensors, and coordinates</h1><p className="intro">Use one active sensor to see how update rate, measurement noise, and coordinate transforms change observations without changing the drone’s true path.</p></header>
    <section className="workspace">
      <div className="panel visual-panel">
        <div className="panel-heading"><div><h2>World view</h2><p>Equal-scale Cartesian view · x rightward · y upward · units in meters</p></div><span className="status-pill">{result ? 'Run loaded' : 'Awaiting run'}</span></div>
        <SimulationCanvas result={result} time={time} showTruth={showTruth} showObservations={showObservations} showTrack={showTrack} showUncertainty={showUncertainty} showNoiseOverlay={showNoiseOverlay} />
        <div className="legend"><span className={`truth ${showTruth ? '' : 'legend-off'}`}>● True path</span><span className={`obs ${showObservations ? '' : 'legend-off'}`}>● Sensor observations</span>{visualConfig.sensor_type === 'cartesian_position' && <><span className={`estimate ${showTrack ? '' : 'legend-off'}`}>╌ Estimated track</span><span className={`track-velocity ${showTrack ? '' : 'legend-off'}`}>→ Estimated velocity</span><span className={`uncertainty ${showUncertainty ? '' : 'legend-off'}`}>◯ 95% position ellipse</span></>}{visualConfig.sensor_type === 'range_bearing' && <span className="sensor-legend">◆ Radar sensor + heading</span>}<span className="velocity">→ True velocity</span>{showNoiseOverlay && <span className="noise-legend">⌁ Noise illustration (σ, not bounds)</span>}</div>
        <p className="scale-note">Velocity arrow scale: 0.2 seconds of travel at the current velocity. The viewport fits the full run and stays fixed while playback moves.</p>
      </div>
      <aside className="panel controls">
        <div className="panel-heading"><div><h2>Scenario</h2><p>{currentScenario.description}</p></div></div>
        <label>Flight pattern<select value={scenarioId} onChange={chooseScenario}>{Object.entries(scenarios).map(([id, scenario]) => <option key={id} value={id}>{scenario.label}</option>)}</select></label>
        <label>Learning preset<select defaultValue="" onChange={applyLearningPreset}><option value="">Choose an experiment…</option>{Object.entries(learningPresets).map(([id, preset]) => <option key={id} value={id}>{preset.label}</option>)}</select><small className="field-help">Presets change controls; run the simulation to apply one.</small></label>
        <label>Tracking preset<select defaultValue="" onChange={applyTrackingPreset}><option value="">Choose a tracking experiment…</option>{Object.entries(trackingPresets).map(([id, preset]) => <option key={id} value={id}>{preset.label}</option>)}</select><small className="field-help">Compare model assumptions, outages, and turns.</small></label>
        <label>Active sensor<select value={config.sensor_type} onChange={(event) => { const sensor_type = event.target.value as SimulationConfig['sensor_type']; setConfig({ ...config, sensor_type, measurement_bias_x: 0, measurement_bias_y: 0 }); }}><option value="cartesian_position">Cartesian position sensor</option><option value="range_bearing">Radar-style range / bearing sensor</option></select></label>
        <p className="tracker-status">{config.sensor_type === 'range_bearing' ? 'Tracking unavailable in radar mode; observations remain sensor-only.' : 'Cartesian tracker: causal estimates, velocity, and model-based uncertainty are shown only after the run is completed.'}</p>
        <div className="control-grid">
          <label>Initial speed (m/s)<input type="number" min="0" step="any" value={config.initial_speed} onChange={update('initial_speed')} /></label>
          <label>Initial heading (°)<input type="number" min="0" max="359.9" step="any" value={config.initial_heading_degrees} onChange={update('initial_heading_degrees')} /></label>
          {config.sensor_type === 'cartesian_position' ? <><label>Noise σx (m)<input type="number" min="0" step="any" value={config.measurement_noise_std_x} onChange={update('measurement_noise_std_x')} /><small className="field-help">Standard deviation; covariance is σ² in m².</small></label><label>Noise σy (m)<input type="number" min="0" step="any" value={config.measurement_noise_std_y} onChange={update('measurement_noise_std_y')} /><small className="field-help">Standard deviation; covariance is σ² in m².</small></label><label>Bias x (m)<input type="number" step="any" value={config.measurement_bias_x} onChange={update('measurement_bias_x')} /></label><label>Bias y (m)<input type="number" step="any" value={config.measurement_bias_y} onChange={update('measurement_bias_y')} /></label></> : <><label>Range noise σ (m)<input type="number" min="0" step="any" value={config.range_noise_std_meters} onChange={update('range_noise_std_meters')} /><small className="field-help">Range variation is shown as dashed circles.</small></label><label>Bearing noise σ (°)<input type="number" min="0" step="any" value={config.bearing_noise_std_degrees} onChange={update('bearing_noise_std_degrees')} /><small className="field-help">Local angle variation; stored in radians.</small></label><label>Sensor heading (world °)<input type="number" step="any" value={config.sensor_heading_degrees} onChange={update('sensor_heading_degrees')} /><small className="field-help">Counterclockwise from world +x.</small></label></>}
          <label>Sensor interval (s)<input type="number" min="0.01" step="any" value={config.sensor_interval_seconds} onChange={update('sensor_interval_seconds')} /><small className="field-help">Update rate: {(1 / config.sensor_interval_seconds).toFixed(2)} Hz</small></label>
          <label>Tracker q (m²/s³)<input type="number" min="0" step="any" value={config.tracker_acceleration_noise_spectral_density} onChange={update('tracker_acceleration_noise_spectral_density')} /><small className="field-help">Motion-model uncertainty: higher q responds more to variation.</small></label>
          <label>Initial velocity σ (m/s)<input type="number" min="0" step="any" value={config.tracker_initial_velocity_std_mps} onChange={update('tracker_initial_velocity_std_mps')} /><small className="field-help">Uncertainty before velocity estimates settle.</small></label>
          <label>Random seed<input type="number" min="0" step="1" value={config.random_seed} onChange={update('random_seed')} /></label>
          <label>Sensor x (m)<input type="number" step="any" value={config.sensor_position_x} onChange={update('sensor_position_x')} /></label>
          <label>Sensor y (m)<input type="number" step="any" value={config.sensor_position_y} onChange={update('sensor_position_y')} /></label>
        </div>
        <p className="sensor-id">One active sensor: <strong>{config.sensor_id}</strong> · world position ({config.sensor_position_x}, {config.sensor_position_y}) m</p>
        <div className="toggle-list"><label className="toggle"><input type="checkbox" checked={showTruth} onChange={(event) => setShowTruth(event.target.checked)} /><span>Show true path</span></label><label className="toggle"><input type="checkbox" checked={showObservations} onChange={(event) => setShowObservations(event.target.checked)} /><span>Show observations</span></label><label className="toggle"><input type="checkbox" checked={showTrack} disabled={config.sensor_type === 'range_bearing'} onChange={(event) => setShowTrack(event.target.checked)} /><span>Show estimated track</span></label><label className="toggle"><input type="checkbox" checked={showUncertainty} disabled={config.sensor_type === 'range_bearing'} onChange={(event) => setShowUncertainty(event.target.checked)} /><span>Show 95% uncertainty ellipse</span></label><label className="toggle"><input type="checkbox" checked={showNoiseOverlay} onChange={(event) => setShowNoiseOverlay(event.target.checked)} /><span>Show noise illustration</span></label></div>
        <div className="turn-settings"><h3>Turn settings</h3>{config.turn_events.length === 0 ? <p className="muted">No turns: the drone flies straight.</p> : config.turn_events.map((event, index) => <div className="turn-card" key={`${event.start_time_seconds}-${index}`}><strong>Turn {index + 1}</strong><span>{event.start_time_seconds.toFixed(1)}–{(event.start_time_seconds + event.duration_seconds).toFixed(1)} s</span><span className={event.turn_rate_degrees_per_second >= 0 ? 'ccw' : 'cw'}>{event.turn_rate_degrees_per_second > 0 ? '+' : ''}{event.turn_rate_degrees_per_second}°/s · {event.turn_rate_degrees_per_second >= 0 ? 'counterclockwise' : 'clockwise'}</span></div>)}</div>
        <div className="outage-settings"><div className="section-heading"><h3>Sensor outage windows</h3><button className="small-button" type="button" onClick={addOutage}>Add outage</button></div>{config.outage_windows.length === 0 ? <p className="muted">Measurements are available at every scheduled time.</p> : config.outage_windows.map((window, index) => <div className="outage-card" key={`${index}-${window.start_time_seconds}`}><label>Start (s)<input type="number" min="0" step="any" value={window.start_time_seconds} onChange={(event) => updateOutage(index, 'start_time_seconds', event)} /></label><label>End (s)<input type="number" min="0" step="any" value={window.end_time_seconds} onChange={(event) => updateOutage(index, 'end_time_seconds', event)} /></label><button className="remove-button" type="button" onClick={() => removeOutage(index)} aria-label={`Remove outage ${index + 1}`}>Remove</button></div>)}<p className="field-help">Samples in [start, end) are omitted; true motion continues and the schedule resumes at the next sample.</p></div>
        <div className="actions"><button className="primary" onClick={runSimulation}>{result ? 'Rerun simulation' : 'Run simulation'}</button><button className="secondary" onClick={resetPlayback} disabled={!result}>Reset playback</button></div><p className="action-help"><strong>Rerun</strong> creates new batch data from the controls. <strong>Reset playback</strong> only rewinds the current run.</p>{error && <p className="error">{error}</p>}
      </aside>
    </section>
    <section className="panel playback-panel">
      <div className="playback-header"><div><h2>Playback</h2><p>Playback uses simulation timestamps; observations appear only at their availability time.</p></div><button className="play-button" onClick={() => setPlaying(!playing)} disabled={!result}>{playing ? 'Pause' : 'Play'}</button></div>
      <input className="timeline" type="range" min="0" max={result?.configuration.duration_seconds ?? 1} step="0.01" value={time} onChange={(event) => { setPlaying(false); setTime(Number(event.target.value)); }} disabled={!result} aria-label="Simulation timeline" />
      <div className="timeline-labels"><span>0.00 s</span><span>{result ? `${time.toFixed(2)} / ${result.configuration.duration_seconds.toFixed(2)} s` : 'Run a scenario first'}</span><span>{result ? `${result.configuration.duration_seconds.toFixed(2)} s` : '—'}</span></div>
      <div className="readout"><h3>Current truth state</h3>{currentState ? <div className="readout-grid"><span><small>time</small>{currentState.timestamp.toFixed(2)} s</span><span><small>x</small>{currentState.x.toFixed(2)} m</span><span><small>y</small>{currentState.y.toFixed(2)} m</span><span><small>vx</small>{currentState.vx.toFixed(2)} m/s</span><span><small>vy</small>{currentState.vy.toFixed(2)} m/s</span><span><small>speed</small>{Math.hypot(currentState.vx, currentState.vy).toFixed(2)} m/s</span><span><small>heading</small>{formatHeading(currentState, config.initial_heading_degrees)}</span></div> : <p className="muted">Run a scenario to inspect its state.</p>}</div>
      <div className="tracker-card"><h3>Causal Cartesian tracker</h3>{visualConfig.sensor_type === 'range_bearing' ? <p className="muted">Unavailable in radar mode. Radar observations remain sensor-only.</p> : latestEstimate ? <><div className="measurement-grid"><span><small>track time</small>{latestEstimate.timestamp.toFixed(2)} s</span><span><small>track x</small>{latestEstimate.estimated_position[0].toFixed(2)} m</span><span><small>track y</small>{latestEstimate.estimated_position[1].toFixed(2)} m</span><span><small>measurement age</small>{(latestEstimate.measurement_age_seconds ?? 0).toFixed(2)} s</span></div><p>{latestEstimate.measurement_updated ? 'Updated with a Cartesian observation at this timestamp.' : 'Prediction only: no measurement was available at this timestamp.'} This is a causal estimate, not truth.</p></> : <p className="muted">The tracker is uninitialized until the first Cartesian observation is available.</p>}</div>
      <div className="measurement-card"><h3>Latest available observation</h3>{latestObservation ? latestObservation.measurement_type === 'range_bearing' ? <><div className="measurement-grid"><span><small>range</small>{latestObservation.measurement_values[0].toFixed(2)} m</span><span><small>local bearing</small>{(latestObservation.measurement_values[1] * 180 / Math.PI).toFixed(2)}°</span><span><small>sensor heading</small>{normalizeDegrees(visualConfig.sensor_heading_degrees).toFixed(2)}° world</span><span><small>measured world heading</small>{normalizeDegrees(visualConfig.sensor_heading_degrees + latestObservation.measurement_values[1] * 180 / Math.PI).toFixed(2)}°</span></div><p>At {latestObservation.measurement_timestamp.toFixed(2)} s: radar bearing is local to the sensor; the converted dot uses the shared world frame. Measured world position: ({measurementToWorldCoordinates(latestObservation, visualConfig).map((value) => value.toFixed(2)).join(', ')}) m.</p></> : <><div className="measurement-grid"><span><small>measured x</small>{latestObservation.measurement_values[0].toFixed(2)} m</span><span><small>measured y</small>{latestObservation.measurement_values[1].toFixed(2)} m</span><span><small>timestamp</small>{latestObservation.measurement_timestamp.toFixed(2)} s</span></div><p>Cartesian observation in the shared world frame.</p></> : <p className="muted">No observation is available at the current playback time.</p>}<p className="measurement-note">Displayed dots and lines are observations, not tracker estimates.</p></div>
      <div className="evaluation"><h3>Ground-truth evaluation</h3>{evaluation ? <p>Latest available observation at <strong>{evaluation.observation.measurement_timestamp.toFixed(2)} s</strong>: position error <strong>{evaluation.distance.toFixed(2)} m</strong> (Δx {evaluation.errorX.toFixed(2)} m, Δy {evaluation.errorY.toFixed(2)} m).</p> : <p className="muted">No observation is available at the current playback time.</p>}<h3 className="metrics-heading">Available-observation metrics</h3>{metrics && metrics.observationCount > 0 ? <div className="metrics-grid"><span><small>mean x error</small>{metrics.meanErrorX.toFixed(2)} m</span><span><small>mean y error</small>{metrics.meanErrorY.toFixed(2)} m</span><span><small>position RMSE</small>{metrics.positionRmse.toFixed(2)} m</span><span><small>observations</small>{metrics.observationCount}</span></div> : <p className="muted">No available observations to evaluate.</p>}<p className="evaluation-note">Errors convert radar measurements to world x/y, then compare each available observation with truth at its measurement timestamp. This is possible because the simulator has truth; a real sensor does not provide it.</p><div className="tracker-evaluation"><h3>Tracker vs. held-measurement baseline</h3>{visualConfig.sensor_type === 'range_bearing' ? <p className="muted">Unavailable in radar mode because tracking accepts Cartesian observations only.</p> : <><label className="warmup-control">Exclude initialization warm-up (s)<input type="number" min="0" max={result?.configuration.duration_seconds ?? 0} step="0.1" value={warmupSeconds} onChange={(event) => setWarmupSeconds(Math.max(0, Number(event.target.value)))} /></label>{trackerMetrics && trackerMetrics.trackerObservationCount > 0 ? <><div className="metrics-grid"><span><small>tracker RMSE</small>{trackerMetrics.trackerPositionRmse.toFixed(2)} m</span><span><small>held-measurement RMSE</small>{trackerMetrics.baselinePositionRmse.toFixed(2)} m</span><span><small>evaluation period</small>{trackerMetrics.evaluationStartSeconds.toFixed(1)}–{trackerMetrics.evaluationEndSeconds.toFixed(1)} s</span><span><small>output timestamps</small>{trackerMetrics.trackerObservationCount}</span></div><p className="evaluation-note">Same output timestamps, from {trackerMetrics.evaluationStartSeconds.toFixed(1)} s through {trackerMetrics.evaluationEndSeconds.toFixed(1)} s reached in playback. The baseline holds the latest available measured position. Truth is used only for this simulator evaluation.</p></> : <p className="muted">No reached output timestamps remain after the selected warm-up.</p>}</>}</div></div>
    </section>
    <section className="learning-notes"><h2>Learning experiments</h2><div className="notes-grid"><p><strong>1 · Same motion, slower updates.</strong> Choose the slower-updates preset and compare the same truth path with fewer observations.</p><p><strong>2 · Same motion, more noise.</strong> Keep the seed fixed, choose more measurement noise, and compare dot scatter without changing motion.</p><p><strong>3 · Same bearing uncertainty, farther target.</strong> Choose the farther-radar preset. The same angular error creates a larger sideways position error at greater distance.</p><p>More frequent measurements and more accurate measurements are different controls.</p><p>Sensor-local coordinates must be transformed into a shared world frame before comparing positions.</p><p>Rotating the radar changes local bearing, not the true path. Noise overlays illustrate configured σ, not guaranteed bounds or tracker confidence regions.</p></div></section>
    <p className="note">This milestone is a simplified educational measurement and tracking model. It omits Doppler, clutter, missed detections, latency, field-of-view and detection-range rules, delayed measurements, track lifecycle management, radar tracking, and multi-sensor fusion.</p>
  </main>;
}
