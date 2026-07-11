/**
 * TypeScript port of sim/agent_sim.py -- 2D agent + environment model used
 * to drive the FSM controller. See that file for the design rationale
 * (event-rate bucket as a looming/optic-flow proxy, not a pixel-level
 * event-camera model).
 */

import { BRAKE_S, FORWARD, TURN_L, TURN_R, type FsmState } from "./fsm.ts";

export const ACTION_NAMES: Record<FsmState, string> = {
  [FORWARD]: "FORWARD",
  [TURN_L]: "TURN_L",
  [TURN_R]: "TURN_R",
  [BRAKE_S]: "BRAKE",
};

export interface Obstacle {
  x: number;
  y: number;
  radius: number;
}

export interface Agent {
  x: number;
  y: number;
  heading: number; // radians, 0 = +x axis, positive = counterclockwise (left)
  speed: number;
  turnRate: number; // radians/step while turning
  radius: number;
}

export function makeAgent(overrides: Partial<Agent> = {}): Agent {
  return {
    x: 0,
    y: 0,
    heading: 0,
    speed: 1.0,
    turnRate: (15 * Math.PI) / 180,
    radius: 0.3,
    ...overrides,
  };
}

export interface Environment {
  obstacles: Obstacle[];
  fov: number; // total field of view, radians
  senseRange: number;
}

export function makeEnvironment(overrides: Partial<Environment> = {}): Environment {
  return {
    obstacles: [],
    fov: (90 * Math.PI) / 180,
    senseRange: 12.0,
    ...overrides,
  };
}

function wrapAngle(angle: number): number {
  return ((angle + Math.PI) % (2 * Math.PI)) - Math.PI;
}

function bucket(intensity: number): number {
  if (intensity >= 0.85) return 3;
  if (intensity >= 0.55) return 2;
  if (intensity >= 0.25) return 1;
  return 0;
}

/** Returns [evLeft, evRight] event-rate buckets in [0, 3]. */
export function sense(agent: Agent, env: Environment): [number, number] {
  const halfFov = env.fov / 2;
  let leftIntensity = 0;
  let rightIntensity = 0;

  for (const obs of env.obstacles) {
    const dx = obs.x - agent.x;
    const dy = obs.y - agent.y;
    const dist = Math.max(Math.hypot(dx, dy) - obs.radius - agent.radius, 0);
    if (dist > env.senseRange) continue;

    const bearing = wrapAngle(Math.atan2(dy, dx) - agent.heading);
    if (Math.abs(bearing) > halfFov) continue;

    const proximity = 1 - dist / env.senseRange;
    const intensity = proximity ** 2;

    if (bearing >= 0) leftIntensity = Math.max(leftIntensity, intensity);
    else rightIntensity = Math.max(rightIntensity, intensity);
  }

  return [bucket(leftIntensity), bucket(rightIntensity)];
}

export function applyAction(agent: Agent, action: FsmState): void {
  switch (action) {
    case FORWARD:
      agent.x += agent.speed * Math.cos(agent.heading);
      agent.y += agent.speed * Math.sin(agent.heading);
      break;
    case TURN_L:
      agent.heading = wrapAngle(agent.heading + agent.turnRate);
      agent.x += 0.5 * agent.speed * Math.cos(agent.heading);
      agent.y += 0.5 * agent.speed * Math.sin(agent.heading);
      break;
    case TURN_R:
      agent.heading = wrapAngle(agent.heading - agent.turnRate);
      agent.x += 0.5 * agent.speed * Math.cos(agent.heading);
      agent.y += 0.5 * agent.speed * Math.sin(agent.heading);
      break;
    case BRAKE_S:
      break; // hold position
  }
}

export function checkCollision(agent: Agent, env: Environment): Obstacle | null {
  for (const obs of env.obstacles) {
    if (Math.hypot(obs.x - agent.x, obs.y - agent.y) <= obs.radius + agent.radius) {
      return obs;
    }
  }
  return null;
}
