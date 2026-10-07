import type { TargetState } from '../types/models';

const EPSILON = 1e-9;

export function stateAtTime(history: TargetState[], time: number): TargetState {
  if (history.length === 0) throw new Error('A simulation must contain truth history');
  if (time <= history[0].timestamp) return history[0];
  for (let index = 1; index < history.length; index += 1) {
    const next = history[index];
    const previous = history[index - 1];
    if (next.timestamp >= time) {
      const span = next.timestamp - previous.timestamp;
      const fraction = span > EPSILON ? (time - previous.timestamp) / span : 0;
      return {...previous, timestamp:time, x:previous.x+(next.x-previous.x)*fraction, y:previous.y+(next.y-previous.y)*fraction, vx:previous.vx+(next.vx-previous.vx)*fraction, vy:previous.vy+(next.vy-previous.vy)*fraction};
    }
  }
  return history[history.length - 1];
}
