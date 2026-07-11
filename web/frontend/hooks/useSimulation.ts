import { useCallback, useEffect, useRef, useState } from "react";
import type { Environment } from "../../engine/agentSim.ts";
import { TsFsmEngine } from "../../engine/FsmEngine.ts";
import { SimulationSession } from "../../engine/simulationLoop.ts";

const DEFAULT_MAX_STEPS = 90;

export function useSimulation(initialEnv: Environment) {
  const engineRef = useRef(new TsFsmEngine());
  const sessionRef = useRef(new SimulationSession(initialEnv, engineRef.current));
  const [tick, setTick] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speedMs, setSpeedMs] = useState(120);

  const step = useCallback(() => {
    const session = sessionRef.current;
    if (session.finished || session.stepCount >= DEFAULT_MAX_STEPS) {
      setPlaying(false);
      return;
    }
    session.step();
    setTick((t) => t + 1);
  }, []);

  useEffect(() => {
    if (!playing) return;
    const id = setInterval(step, speedMs);
    return () => clearInterval(id);
  }, [playing, speedMs, step]);

  const reset = useCallback((env: Environment) => {
    setPlaying(false);
    sessionRef.current = new SimulationSession(env, engineRef.current);
    setTick((t) => t + 1);
  }, []);

  return {
    session: sessionRef.current,
    engineName: engineRef.current.name,
    tick,
    playing,
    setPlaying,
    speedMs,
    setSpeedMs,
    step,
    reset,
    maxSteps: DEFAULT_MAX_STEPS,
  };
}
