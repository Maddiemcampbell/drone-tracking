import {useEffect,useState} from 'react';
import type {ChangeEvent} from 'react';
import {SimulationCanvas} from './components/SimulationCanvas';
import {simulate} from './simulation/api';
import type {SimulationConfig,SimulationResult} from './types/models';
import './styles.css';
const defaults:SimulationConfig={duration_seconds:20,simulation_timestep_seconds:.1,sensor_interval_seconds:.5,measurement_noise_std:5,random_seed:7,initial_x:0,initial_y:0,initial_vx:10,initial_vy:5};
export default function App(){
 const[config,setConfig]=useState(defaults),[result,setResult]=useState<SimulationResult|null>(null),[time,setTime]=useState(0),[playing,setPlaying]=useState(false),[error,setError]=useState('');
 const update=(key:keyof SimulationConfig)=>(e:ChangeEvent<HTMLInputElement>)=>setConfig({...config,[key]:Number(e.target.value)});
 useEffect(()=>{if(!playing||!result)return;const id=setInterval(()=>setTime(t=>{const next=t+.05;if(next>=result.configuration.duration_seconds){setPlaying(false);return result.configuration.duration_seconds}return next}),50);return()=>clearInterval(id)},[playing,result]);
 async function run(){setError('');try{setResult(await simulate(config));setTime(0);setPlaying(false)}catch(e){setError(e instanceof Error?e.message:'Unable to run simulation')}}
 return <main><h1>Sensor Tracking Simulator</h1><p className="intro">A hardware-free learning model: one target, constant velocity, and noisy Cartesian position observations. The dashed line is a constant-velocity Kalman estimate based on observations only.</p><section className="layout"><div className="panel controls"><h2>Run configuration</h2>{(['measurement_noise_std','sensor_interval_seconds','random_seed'] as const).map(key=><label key={key}>{key.replace(/_/g,' ')}<input type="number" min="0" step="any" value={config[key]} onChange={update(key)}/></label>)}<button onClick={run}>Run simulation</button>{result&&<div className="buttons"><button onClick={()=>setPlaying(!playing)}>{playing?'Pause':'Play'}</button><button onClick={()=>{setTime(0);setPlaying(false)}}>Reset</button></div>}{error&&<p className="error">{error}</p>}</div><div className="panel"><SimulationCanvas result={result} time={time}/>{result&&<p className="time">Simulation time: {time.toFixed(2)}s / {result.configuration.duration_seconds}s</p>}<div className="legend"><span className="truth">● Truth path</span><span className="obs">● Noisy observations</span><span className="estimate">╌ Kalman estimate</span></div></div></section><p className="note">Truth is the actual simulated state; the tracker sees observations only. The dashed estimate is intentionally simple and is the first state-estimation milestone.</p></main>
}
