/**
 * Engine abstraction so the UI never talks to fsm.ts directly. Today the
 * only implementation is TsFsmEngine, a plain TypeScript reference model.
 * A future WasmFsmEngine (rtl/fsm_controller.sv compiled via Verilator +
 * Emscripten) can implement the same interface and be dropped in as an
 * alternate, selectable engine without any UI changes -- see
 * docs/architecture.md in the repo root for the plan.
 */

import { DEFAULT_FSM_PARAMS, stepFsm, type FsmParams, type FsmStepResult, type FsmState } from "./fsm.ts";

export interface FsmEngine {
  readonly name: string;
  step(state: FsmState, brakeTimer: number, evLeft: number, evRight: number): FsmStepResult;
}

export class TsFsmEngine implements FsmEngine {
  readonly name = "TypeScript reference model";

  constructor(private params: FsmParams = DEFAULT_FSM_PARAMS) {}

  step(state: FsmState, brakeTimer: number, evLeft: number, evRight: number): FsmStepResult {
    return stepFsm(state, brakeTimer, evLeft, evRight, this.params);
  }
}
