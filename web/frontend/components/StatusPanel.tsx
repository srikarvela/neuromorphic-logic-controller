import { ACTION_NAMES } from "../../engine/agentSim.ts";
import type { FsmState } from "../../engine/fsm.ts";

interface Props {
  engineName: string;
  state: FsmState;
  brakeTimer: number;
  evLeft: number;
  evRight: number;
  stepCount: number;
  maxSteps: number;
  collided: boolean;
}

function EventBar({ label, value }: { label: string; value: number }) {
  return (
    <div className="event-bar">
      <span>{label}</span>
      <div className="event-bar-track">
        <div className="event-bar-fill" style={{ width: `${(value / 3) * 100}%` }} />
      </div>
      <span>{value}</span>
    </div>
  );
}

export function StatusPanel({
  engineName,
  state,
  brakeTimer,
  evLeft,
  evRight,
  stepCount,
  maxSteps,
  collided,
}: Props) {
  return (
    <div className="status-panel">
      <div className={`state-badge state-${ACTION_NAMES[state]}`}>{ACTION_NAMES[state]}</div>
      <EventBar label="ev_left" value={evLeft} />
      <EventBar label="ev_right" value={evRight} />
      <div className="status-row">
        <span>brake_timer: {brakeTimer}</span>
        <span>
          step {stepCount}/{maxSteps}
        </span>
      </div>
      <div className="status-row engine-name">engine: {engineName}</div>
      {collided && <div className="collision-banner">Collision</div>}
    </div>
  );
}
