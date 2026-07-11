interface Props {
  playing: boolean;
  onTogglePlay: () => void;
  onStep: () => void;
  onResetSame: () => void;
  onRandomize: () => void;
  speedMs: number;
  onSpeedChange: (ms: number) => void;
  seed: number;
}

export function ControlPanel({
  playing,
  onTogglePlay,
  onStep,
  onResetSame,
  onRandomize,
  speedMs,
  onSpeedChange,
  seed,
}: Props) {
  return (
    <div className="control-panel">
      <button onClick={onTogglePlay}>{playing ? "Pause" : "Play"}</button>
      <button onClick={onStep} disabled={playing}>
        Step
      </button>
      <button onClick={onResetSame}>Reset course</button>
      <button onClick={onRandomize}>Randomize course</button>

      <label className="speed-control">
        Speed
        <input
          type="range"
          min={30}
          max={400}
          step={10}
          value={420 - speedMs}
          onChange={(e) => onSpeedChange(420 - Number(e.target.value))}
        />
      </label>

      <span className="seed-label">seed {seed}</span>
    </div>
  );
}
