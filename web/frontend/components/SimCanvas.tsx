import { useEffect, useRef } from "react";
import type { Agent, Environment } from "../../engine/agentSim.ts";
import { BRAKE_S, TURN_L, TURN_R, type FsmState } from "../../engine/fsm.ts";

interface Props {
  agent: Agent;
  env: Environment;
  trail: { x: number; y: number }[];
  state: FsmState;
  collided: boolean;
  width?: number;
  height?: number;
}

const STATE_COLOR: Record<FsmState, string> = {
  0: "#5ee6a0",
  [TURN_L]: "#f4c95d",
  [TURN_R]: "#f4c95d",
  [BRAKE_S]: "#f45d5d",
};

/** World -> canvas transform that auto-fits obstacles + trail + agent, so both the default and randomized courses render sensibly without hardcoded bounds. */
function fitTransform(
  points: { x: number; y: number }[],
  canvasWidth: number,
  canvasHeight: number,
  margin = 3,
) {
  let minX = -2;
  let maxX = 10;
  let minY = -5;
  let maxY = 5;
  if (points.length > 0) {
    minX = Math.min(...points.map((p) => p.x)) - margin;
    maxX = Math.max(...points.map((p) => p.x)) + margin;
    minY = Math.min(...points.map((p) => p.y)) - margin;
    maxY = Math.max(...points.map((p) => p.y)) + margin;
  }
  const worldW = Math.max(maxX - minX, 1);
  const worldH = Math.max(maxY - minY, 1);
  const scale = Math.min(canvasWidth / worldW, canvasHeight / worldH);
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;

  return {
    toCanvas(x: number, y: number): [number, number] {
      return [canvasWidth / 2 + (x - centerX) * scale, canvasHeight / 2 - (y - centerY) * scale];
    },
    scale,
  };
}

export function SimCanvas({ agent, env, trail, state, collided, width = 860, height = 460 }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const points = [...env.obstacles, ...trail, { x: agent.x, y: agent.y }];
    const view = fitTransform(points, width, height);

    ctx.fillStyle = "#0d1117";
    ctx.fillRect(0, 0, width, height);

    // Obstacles
    for (const obs of env.obstacles) {
      const [cx, cy] = view.toCanvas(obs.x, obs.y);
      ctx.beginPath();
      ctx.arc(cx, cy, obs.radius * view.scale, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(244, 93, 93, 0.35)";
      ctx.fill();
      ctx.strokeStyle = "rgba(244, 93, 93, 0.8)";
      ctx.stroke();
    }

    // Trail
    if (trail.length > 1) {
      ctx.beginPath();
      trail.forEach((p, i) => {
        const [cx, cy] = view.toCanvas(p.x, p.y);
        if (i === 0) ctx.moveTo(cx, cy);
        else ctx.lineTo(cx, cy);
      });
      ctx.strokeStyle = "#6ea8fe";
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Agent (triangle pointing along heading)
    const [ax, ay] = view.toCanvas(agent.x, agent.y);
    const size = Math.max(agent.radius * view.scale, 6);
    ctx.save();
    ctx.translate(ax, ay);
    ctx.rotate(-agent.heading);
    ctx.beginPath();
    ctx.moveTo(size * 1.6, 0);
    ctx.lineTo(-size, size);
    ctx.lineTo(-size, -size);
    ctx.closePath();
    ctx.fillStyle = collided ? "#f45d5d" : STATE_COLOR[state];
    ctx.fill();
    ctx.restore();

    if (collided) {
      ctx.beginPath();
      ctx.arc(ax, ay, size * 2.2, 0, Math.PI * 2);
      ctx.strokeStyle = "#f45d5d";
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }, [agent, env, trail, state, collided, width, height]);

  return <canvas ref={canvasRef} className="sim-canvas" />;
}
