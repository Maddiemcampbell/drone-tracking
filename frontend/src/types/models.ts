export type TurnEvent = { start_time_seconds:number; duration_seconds:number; turn_rate_degrees_per_second:number };
export type SimulationConfig = { duration_seconds:number; simulation_timestep_seconds:number; sensor_interval_seconds:number; measurement_noise_std_x:number; measurement_noise_std_y:number; random_seed:number; sensor_id:string; sensor_position_x:number; sensor_position_y:number; initial_x:number; initial_y:number; initial_speed:number; initial_heading_degrees:number; turn_events:TurnEvent[] };
export type TargetState = { target_id:string; timestamp:number; x:number; y:number; vx:number; vy:number };
export type SensorObservation = { sensor_id:string; measurement_timestamp:number; availability_timestamp:number; measurement_type:'cartesian_position'; measurement_values:number[]; measurement_covariance:number[][]; sensor_position:number[] };
export type TrackEstimate = { track_id:string; timestamp:number; estimated_position:number[]; estimated_velocity:number[]; state_covariance:number[][] };
export type SimulationResult = { configuration:SimulationConfig; truth_history:TargetState[]; observations:SensorObservation[]; estimates:TrackEstimate[] };
