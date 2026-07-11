import { ACTION_NAMES, applyAction, checkCollision, makeAgent, sense, type Agent, type Environment, type Obstacle } from "./agentSim.ts";
import { FORWARD, type FsmState } from "./fsm.ts";
import type { FsmEngine } from "./FsmEngine.ts";

export interface StepLogRow {
  step: number;
  x: number;
  y: number;
  headingDeg: number;
  evLeft: number;
  evRight: number;
  state: FsmState;
  action: string;
  brakeTimer: number;
}

/**
 * Real-time counterpart to sim/cosim_driver.py::run_episode -- owns the
 * agent/environment and calls engine.step() once per control tick,
 * instead of batch-running to completion.
 */
export class SimulationSession {
  agent: Agent;
  env: Environment;
  engine: FsmEngine;
  state: FsmState = FORWARD;
  brakeTimer = 0;
  stepCount = 0;
  log: StepLogRow[] = [];
  collidedWith: Obstacle | null = null;

  constructor(env: Environment, engine: FsmEngine, agent: Agent = makeAgent()) {
    this.env = env;
    this.engine = engine;
    this.agent = agent;
  }

  get finished(): boolean {
    return this.collidedWith !== null;
  }

  step(): StepLogRow | null {
    if (this.finished) return null;

    const [evLeft, evRight] = sense(this.agent, this.env);
    const result = this.engine.step(this.state, this.brakeTimer, evLeft, evRight);
    this.state = result.state;
    this.brakeTimer = result.brakeTimer;
    applyAction(this.agent, this.state);

    const hit = checkCollision(this.agent, this.env);
    if (hit) this.collidedWith = hit;

    const row: StepLogRow = {
      step: this.stepCount,
      x: this.agent.x,
      y: this.agent.y,
      headingDeg: (this.agent.heading * 180) / Math.PI,
      evLeft,
      evRight,
      state: this.state,
      action: ACTION_NAMES[this.state],
      brakeTimer: this.brakeTimer,
    };
    this.log.push(row);
    this.stepCount++;
    return row;
  }
}
