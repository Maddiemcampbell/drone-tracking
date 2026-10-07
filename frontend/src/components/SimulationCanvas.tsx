import { useEffect, useRef } from 'react';
import { stateAtTime } from '../simulation/playback';
import { measurementToWorldCoordinates } from '../simulation/measurement';
import type { TargetState, SimulationResult } from '../types/models';

const VELOCITY_ARROW_SECONDS = 0.2;
const EPSILON = 1e-9;

function drawArrow(ctx: CanvasRenderingContext2D, fromX: number, fromY: number, toX: number, toY: number, color: string) {
  const angle = Math.atan2(toY - fromY, toX - fromX);
  const head = 9;
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(fromX, fromY); ctx.lineTo(toX, toY); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(toX, toY); ctx.lineTo(toX-head*Math.cos(angle-Math.PI/6), toY-head*Math.sin(angle-Math.PI/6)); ctx.lineTo(toX-head*Math.cos(angle+Math.PI/6), toY-head*Math.sin(angle+Math.PI/6)); ctx.closePath(); ctx.fill();
}

export function SimulationCanvas({result,time,showTruth,showObservations}:{result:SimulationResult|null;time:number;showTruth:boolean;showObservations:boolean}) {
  const ref=useRef<HTMLCanvasElement>(null);
  useEffect(()=>{
    const canvas=ref.current,ctx=canvas?.getContext('2d'); if(!canvas||!ctx)return;
    const width=canvas.width,height=canvas.height;ctx.clearRect(0,0,width,height);ctx.fillStyle='#0d1726';ctx.fillRect(0,0,width,height);
    if(!result||result.truth_history.length===0){ctx.fillStyle='#aab8c8';ctx.font='18px system-ui, sans-serif';ctx.fillText('Run a scenario to explore its motion.',32,height/2);return;}
    const truth=result.truth_history,observedWorld=result.observations.map(observation=>measurementToWorldCoordinates(observation,result.configuration)),xs=[...truth.map(s=>s.x),...observedWorld.map(point=>point[0])],ys=[...truth.map(s=>s.y),...observedWorld.map(point=>point[1])],rawMinX=Math.min(...xs),rawMaxX=Math.max(...xs),rawMinY=Math.min(...ys),rawMaxY=Math.max(...ys),rawSpan=Math.max(rawMaxX-rawMinX,rawMaxY-rawMinY,1),margin=Math.max(rawSpan*.08,1),minX=rawMinX-margin,maxX=rawMaxX+margin,minY=rawMinY-margin,maxY=rawMaxY+margin,padding=54,scale=Math.min((width-padding*2)/(maxX-minX),(height-padding*2)/(maxY-minY)),centerX=(minX+maxX)/2,centerY=(minY+maxY)/2;
    const toCanvas=(x:number,y:number):[number,number]=>[width/2+(x-centerX)*scale,height/2-(y-centerY)*scale];
    const axisX=toCanvas(0,0)[0],axisY=toCanvas(0,0)[1];
    const drawAxis=(horizontal:boolean)=>{const inBounds=horizontal?axisY>=padding&&axisY<=height-padding:axisX>=padding&&axisX<=width-padding;const startX=horizontal?padding:(inBounds?axisX:padding),startY=horizontal?(inBounds?axisY:height-padding):height-padding,endX=horizontal?width-padding:startX,endY=horizontal?startY:padding;ctx.strokeStyle='#31445b';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(startX,startY);ctx.lineTo(endX,endY);ctx.stroke();drawArrow(ctx,horizontal?endX-16:startX,horizontal?endY:endY+16,endX,endY,'#71869d');ctx.fillStyle='#8ea1b5';ctx.font='13px system-ui, sans-serif';ctx.fillText(horizontal?'x (m) →':'y (m) ↑',horizontal?endX-48:startX+8,horizontal?endY-8:endY+16);};
    drawAxis(true);drawAxis(false);
    const current=stateAtTime(truth,Math.min(Math.max(time,0),result.configuration.duration_seconds));
    const visibleTruth=truth.filter(state=>state.timestamp<=current.timestamp+EPSILON),path=visibleTruth.length>0?[...visibleTruth]:[truth[0]];if(path[path.length-1].timestamp<current.timestamp-EPSILON)path.push(current);
    const drawPath=(states:TargetState[],color:string,dashed=false)=>{if(states.length===0)return;ctx.strokeStyle=color;ctx.lineWidth=dashed?2:3;ctx.setLineDash(dashed?[7,5]:[]);ctx.beginPath();states.forEach((state,index)=>{const [x,y]=toCanvas(state.x,state.y);if(index===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();ctx.setLineDash([]);};
    if(showTruth) drawPath(path,'#49dcb1');
    if(showObservations){ctx.fillStyle='#ffb454';result.observations.filter(observation=>observation.availability_timestamp<=current.timestamp+EPSILON).forEach(observation=>{const [x,y]=toCanvas(...measurementToWorldCoordinates(observation,result.configuration));ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fill();});}
    const [currentX,currentY]=toCanvas(current.x,current.y),[arrowX,arrowY]=toCanvas(current.x+current.vx*VELOCITY_ARROW_SECONDS,current.y+current.vy*VELOCITY_ARROW_SECONDS);if(Math.hypot(current.vx,current.vy)>EPSILON)drawArrow(ctx,currentX,currentY,arrowX,arrowY,'#f6f7fb');if(showTruth){ctx.fillStyle='#49dcb1';ctx.strokeStyle='#f6f7fb';ctx.lineWidth=2;ctx.beginPath();ctx.arc(currentX,currentY,9,0,Math.PI*2);ctx.fill();ctx.stroke();}
  },[result,time,showTruth,showObservations]);
  return <canvas ref={ref} width={900} height={560} aria-label="Motion simulation world showing the drone trajectory"/>;
}

export {VELOCITY_ARROW_SECONDS};
