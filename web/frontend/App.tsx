import { useMemo, useState } from "react";
import { defaultEnvironment, randomEnvironment } from "../engine/environments.ts";
import { ControlPanel } from "./components/ControlPanel.tsx";
import { SimCanvas } from "./components/SimCanvas.tsx";
import { StatusPanel } from "./components/StatusPanel.tsx";
import { useSimulation } from "./hooks/useSimulation.ts";

export function App() {
  const [seed, setSeed] = useState<number | "default">("default");
  const initialEnv = useMemo(() => defaultEnvironment(), []);
  const { session, engineName, tick, playing, setPlaying, speedMs, setSpeedMs, step, reset, maxSteps } =
    useSimulation(initialEnv);

  // Read void tick so this component re-renders every simulation step even
  // though `session` is a mutable object whose reference never changes.
  void tick;

  const lastRow = session.log[session.log.length - 1];
  const trail = [{ x: 0, y: 0 }, ...session.log.map((r) => ({ x: r.x, y: r.y }))];

  return (
    <div className="app">
      <header>
        <h1>Neuromorphic Logic Controller</h1>
        <p>
          A reactive obstacle-avoidance FSM, ported from{" "}
          <code>rtl/fsm_controller.sv</code> and parity-tested against the same vectors as the
          SystemVerilog unit testbench. See <code>web/engine/FsmEngine.ts</code> for the swap point
          where a WASM build of the real RTL can replace this reference model later.
        </p>
      </header>

      <main>
        <SimCanvas
          agent={{ ...session.agent }}
          env={session.env}
          trail={trail}
          state={session.state}
          collided={session.finished}
        />

        <div className="sidebar">
          <StatusPanel
            engineName={engineName}
            state={session.state}
            brakeTimer={session.brakeTimer}
            evLeft={lastRow?.evLeft ?? 0}
            evRight={lastRow?.evRight ?? 0}
            stepCount={session.stepCount}
            maxSteps={maxSteps}
            collided={session.finished}
          />
          <ControlPanel
            playing={playing}
            onTogglePlay={() => setPlaying((p) => !p)}
            onStep={step}
            onResetSame={() => reset(session.env)}
            onRandomize={() => {
              const nextSeed = seed === "default" ? 1 : seed + 1;
              setSeed(nextSeed);
              reset(randomEnvironment(nextSeed));
            }}
            speedMs={speedMs}
            onSpeedChange={setSpeedMs}
            seed={seed === "default" ? 0 : seed}
          />
        </div>
      </main>
    </div>
  );
}
