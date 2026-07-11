/**
 * Parity test: replays the exact same directed + edge-case vector sequence
 * as tb/tb_fsm_controller.sv (42 checks, all passing under Icarus Verilog)
 * against the TypeScript reference model. Every step here corresponds
 * 1:1 to one `step_check` call in that testbench, in the same order, so
 * this is a real cross-check of the port -- not just a plausible-looking
 * reimplementation.
 */
import { describe, expect, it } from "vitest";
import { BRAKE_S, FORWARD, TURN_L, TURN_R, stepFsm, type FsmState } from "./fsm.ts";

const BRAKE_CYCLES = 4;

interface Vector {
  evLeft: number;
  evRight: number;
  expectedState: FsmState;
  expectedTimer?: number;
  label: string;
}

// Transcribed 1:1 from tb/tb_fsm_controller.sv's initial block.
const VECTORS: Vector[] = [
  { evLeft: 0, evRight: 0, expectedState: FORWARD, label: "reset -> FORWARD" },
  { evLeft: 0, evRight: 0, expectedState: FORWARD, label: "no events -> FORWARD" },

  { evLeft: 3, evRight: 0, expectedState: TURN_R, label: "obstacle left -> TURN_R" },
  { evLeft: 3, evRight: 0, expectedState: TURN_R, label: "obstacle left persists -> TURN_R holds" },
  { evLeft: 0, evRight: 0, expectedState: FORWARD, label: "obstacle left clears -> FORWARD" },

  { evLeft: 0, evRight: 3, expectedState: TURN_L, label: "obstacle right -> TURN_L" },
  { evLeft: 0, evRight: 0, expectedState: FORWARD, label: "obstacle right clears -> FORWARD" },

  { evLeft: 3, evRight: 3, expectedState: BRAKE_S, expectedTimer: BRAKE_CYCLES, label: "critical front -> BRAKE" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "brake timer countdown 0" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "brake timer countdown 1" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "brake timer countdown 2" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "brake timer countdown 3" },
  { evLeft: 0, evRight: 0, expectedState: FORWARD, label: "brake timer expires -> FORWARD" },

  // Edge case: BRAKE preempts an in-progress TURN_R with a fresh timer.
  { evLeft: 3, evRight: 0, expectedState: TURN_R, label: "obstacle left -> TURN_R (preempt setup)" },
  {
    evLeft: 3,
    evRight: 3,
    expectedState: BRAKE_S,
    expectedTimer: BRAKE_CYCLES,
    label: "critical front mid-TURN_R -> BRAKE preempts turn",
  },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "preempt-from-TURN_R countdown 0" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "preempt-from-TURN_R countdown 1" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "preempt-from-TURN_R countdown 2" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "preempt-from-TURN_R countdown 3" },
  { evLeft: 0, evRight: 0, expectedState: FORWARD, label: "preempt-from-TURN_R brake expires -> FORWARD" },

  // Same edge case, mirrored: BRAKE preempts an in-progress TURN_L.
  { evLeft: 0, evRight: 3, expectedState: TURN_L, label: "obstacle right -> TURN_L (preempt setup)" },
  {
    evLeft: 3,
    evRight: 3,
    expectedState: BRAKE_S,
    expectedTimer: BRAKE_CYCLES,
    label: "critical front mid-TURN_L -> BRAKE preempts turn",
  },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "preempt-from-TURN_L countdown 0" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "preempt-from-TURN_L countdown 1" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "preempt-from-TURN_L countdown 2" },
  { evLeft: 0, evRight: 0, expectedState: BRAKE_S, label: "preempt-from-TURN_L countdown 3" },
  { evLeft: 0, evRight: 0, expectedState: FORWARD, label: "preempt-from-TURN_L brake expires -> FORWARD" },

  // Edge case: obstacle still critical exactly when the timer expires ->
  // one-cycle FORWARD flicker, then straight back into a fresh BRAKE.
  { evLeft: 3, evRight: 3, expectedState: BRAKE_S, expectedTimer: BRAKE_CYCLES, label: "critical sustained -> BRAKE" },
  { evLeft: 3, evRight: 3, expectedState: BRAKE_S, label: "sustained-critical countdown 0" },
  { evLeft: 3, evRight: 3, expectedState: BRAKE_S, label: "sustained-critical countdown 1" },
  { evLeft: 3, evRight: 3, expectedState: BRAKE_S, label: "sustained-critical countdown 2" },
  { evLeft: 3, evRight: 3, expectedState: BRAKE_S, label: "sustained-critical countdown 3" },
  { evLeft: 3, evRight: 3, expectedState: FORWARD, label: "timer expiry blip -> FORWARD even though still critical" },
  {
    evLeft: 3,
    evRight: 3,
    expectedState: BRAKE_S,
    expectedTimer: BRAKE_CYCLES,
    label: "critical still present -> re-enters BRAKE next cycle",
  },
];

describe("stepFsm parity with tb/tb_fsm_controller.sv", () => {
  it("matches all 30 state transitions from the SV testbench, in order", () => {
    let state: FsmState = FORWARD;
    let brakeTimer = 0;

    for (const vector of VECTORS) {
      const result = stepFsm(state, brakeTimer, vector.evLeft, vector.evRight);
      expect(result.state, vector.label).toBe(vector.expectedState);
      if (vector.expectedTimer !== undefined) {
        expect(result.brakeTimer, `${vector.label} (timer)`).toBe(vector.expectedTimer);
      }
      state = result.state;
      brakeTimer = result.brakeTimer;
    }
  });

  it("cmd_* outputs always match the resulting state one-hot", () => {
    let state: FsmState = FORWARD;
    let brakeTimer = 0;
    for (const vector of VECTORS) {
      const result = stepFsm(state, brakeTimer, vector.evLeft, vector.evRight);
      expect(result.cmdForward).toBe(result.state === FORWARD);
      expect(result.cmdTurnLeft).toBe(result.state === TURN_L);
      expect(result.cmdTurnRight).toBe(result.state === TURN_R);
      expect(result.cmdBrake).toBe(result.state === BRAKE_S);
      state = result.state;
      brakeTimer = result.brakeTimer;
    }
  });
});
