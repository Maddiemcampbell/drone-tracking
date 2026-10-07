import { useEffect, useRef } from 'react';
import type { SimulationResult } from '../types/models';

export function SimulationCanvas({result,time}:{result:SimulationResult|null;time:number}) {
  const ref=useRef<HTMLCanvasElement>(null);
  useEffect(()=>{
    const canvas=ref.current;if(!canvas||!result)return;const ctx=canvas.getContext('2d');if(!ctx)return;
    const w=canvas.width,h=canvas.height,pad=32;ctx.clearRect(0,0,w,h);ctx.fillStyle='#101827';ctx.fillRect(0,0,w,h);
    const all=result.truth_history,xs=all.map(s=>s.x),ys=all.map(s=>s.y),minX=Math.min(...xs)-10,maxX=Math.max(...xs)+10,minY=Math.min(...ys)-10,maxY=Math.max(...ys)+10;
    const px=(x:number)=>pad+(x-minX)/(maxX-minX)*(w-2*pad),py=(y:number)=>h-pad-(y-minY)/(maxY-minY)*(h-2*pad);
    ctx.strokeStyle='#49dcb1';ctx.lineWidth=2;ctx.beginPath();all.forEach((s,i)=>i?ctx.lineTo(px(s.x),py(s.y)):ctx.moveTo(px(s.x),py(s.y)));ctx.stroke();
    ctx.strokeStyle='#9b8cff';ctx.lineWidth=2;ctx.setLineDash([6,4]);ctx.beginPath();result.estimates.forEach((s,i)=>{if(s.timestamp<=time){const point=s.estimated_position;i?ctx.lineTo(px(point[0]),py(point[1])):ctx.moveTo(px(point[0]),py(point[1]));}});ctx.stroke();ctx.setLineDash([]);
    ctx.fillStyle='#ffb454';result.observations.forEach(o=>{if(o.measurement_timestamp<=time){ctx.beginPath();ctx.arc(px(o.measurement_values[0]),py(o.measurement_values[1]),4,0,Math.PI*2);ctx.fill();}});
    const current=[...all].reverse().find(s=>s.timestamp<=time)??all[0];ctx.fillStyle='#49dcb1';ctx.beginPath();ctx.arc(px(current.x),py(current.y),6,0,Math.PI*2);ctx.fill();
  },[result,time]);
  return <canvas ref={ref} width={760} height={440} aria-label="Simulation plot"/>;
}
