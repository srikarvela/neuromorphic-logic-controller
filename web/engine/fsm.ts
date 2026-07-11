/**
 * TypeScript reference model of rtl/fsm_controller.sv.
 *
 * This must stay behavior-identical to the SystemVerilog FSM -- it is
 * parity-tested against the same directed + edge-case vectors used in
 * tb/tb_fsm_controller.sv (see engine/fsm.test.ts). If you change the RTL's
 * transition logic, update this file and its tests too.
 */

export const FORWARD = 0;
export const TURN_L = 1;
export const TURN_R = 2;
export const BRAKE_S = 3;

export type FsmState = typeof FORWARD | typeof TURN_L | typeof TURN_R | typeof BRAKE_S;

export interface FsmParams {
  obstacleThresh: number;
  brakeThresh: number;
  brakeCycles: number;
}

export const DEFAULT_FSM_PARAMS: FsmParams = {
  obstacleThresh: 2,
  brakeThresh: 3,
  brakeCycles: 4,
};

export interface FsmStepResult {
  state: FsmState;
  brakeTimer: number;
  cmdForward: boolean;
  cmdTurnLeft: boolean;
  cmdTurnRight: boolean;
  cmdBrake: boolean;
}

/**
 * Advance the FSM by exactly one clock edge, mirroring the RTL's
 * always_comb (next-state logic) followed by always_ff (register update).
 * Returned cmd_* flags reflect the *new* registered state, matching what
 * the RTL's combinational outputs would read the cycle after this edge --
 * the same convention tb/tb_cosim_step.sv and sim/cosim_driver.py use.
 */
export function stepFsm(
  state: FsmState,
  brakeTimer: number,
  evLeft: number,
  evRight: number,
  params: FsmParams = DEFAULT_FSM_PARAMS,
): FsmStepResult {
  const { obstacleThresh, brakeThresh, brakeCycles } = params;

  const obstacleLeft = evLeft >= obstacleThresh;
  const obstacleRight = evRight >= obstacleThresh;
  const criticalFront = evLeft >= brakeThresh && evRight >= brakeThresh;

  let nextState: FsmState = state;
  let brakeTimerNext = brakeTimer;

  switch (state) {
    case FORWARD:
      if (criticalFront) nextState = BRAKE_S;
      else if (obstacleLeft) nextState = TURN_R;
      else if (obstacleRight) nextState = TURN_L;
      break;

    case TURN_R:
      if (criticalFront) nextState = BRAKE_S;
      else if (!obstacleLeft) nextState = FORWARD;
      break;

    case TURN_L:
      if (criticalFront) nextState = BRAKE_S;
      else if (!obstacleRight) nextState = FORWARD;
      break;

    case BRAKE_S:
      if (brakeTimer === 0) nextState = FORWARD;
      else brakeTimerNext = brakeTimer - 1;
      break;
  }

  // Load the timer the cycle we (re-)enter BRAKE_S, same as the RTL.
  if (nextState === BRAKE_S && state !== BRAKE_S) {
    brakeTimerNext = brakeCycles;
  }

  return {
    state: nextState,
    brakeTimer: brakeTimerNext,
    cmdForward: nextState === FORWARD,
    cmdTurnLeft: nextState === TURN_L,
    cmdTurnRight: nextState === TURN_R,
    cmdBrake: nextState === BRAKE_S,
  };
}
