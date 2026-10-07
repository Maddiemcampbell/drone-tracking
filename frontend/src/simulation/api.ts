import type { SimulationConfig, SimulationResult } from '../types/models';
const API = import.meta.env.VITE_API_URL ?? 'http://localhost:8000';
export async function simulate(config: SimulationConfig): Promise<SimulationResult> {
  const response = await fetch(`${API}/api/simulate`, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(config)});
  if (!response.ok) throw new Error('Simulation request failed');
  return response.json() as Promise<SimulationResult>;
}
