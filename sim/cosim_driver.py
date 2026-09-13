"""Hardware-in-the-loop driver.

Python owns the agent/environment physics and a DVS-style event-camera
model; the controller "chip" -- windowed event counters plus the FSM -- is
real RTL, running either under Icarus Verilog or on the PYNQ-Z2's FPGA.
Each control step, one packet of event words goes in and one decision word
comes back. See docs/architecture.md.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from agent_sim import ACTION_NAMES, Agent, Environment, Obstacle, apply_action, check_collision, sense
from engines import Engine, add_engine_args, engine_from_args
from event_camera import EventCamera, EventCameraConfig
from event_stream import Decision, words_to_bytes
from golden_model import GoldenPipeline

ROOT = Path(__file__).resolve().parent.parent


class ParityError(RuntimeError):
    pass


def build_default_environment() -> Environment:
    # Positions are staggered along the agent's own post-avoidance drift path
    # (see docs/architecture.md) so the FSM has to react to each one in turn
    # instead of just missing obstacles it happens to have already dodged.
    return Environment(
        obstacles=[
            Obstacle(x=10.0, y=0.8, radius=1.2),
            Obstacle(x=18.7, y=-6.0, radius=1.3),
            Obstacle(x=24.7, y=-19.9, radius=1.3),
        ],
    )


def check_decision(step: int, hw: Decision, golden: Decision | None, expected_buckets: tuple[int, int] | None) -> None:
    """Cross-check one hardware decision against the golden model and the sensor model."""
    if golden is not None and hw != golden:
        raise ParityError(f"step {step}: hardware decision {hw} != golden model {golden}")
    if expected_buckets is not None and (hw.ev_left, hw.ev_right) != expected_buckets:
        raise ParityError(
            f"step {step}: hardware quantized events to {(hw.ev_left, hw.ev_right)}, "
            f"software rate model says {expected_buckets}"
        )


def run_episode(
    engine: Engine,
    env: Environment | None = None,
    max_steps: int = 200,
    log_path: Path | None = None,
    camera: EventCamera | None = None,
    check: bool = True,
    record: list[int] | None = None,
) -> tuple[list[dict], Environment]:
    """One closed-loop episode. Returns the per-step log and the environment.

    With `check`, every decision is compared against the golden model fed
    the identical packet, and its quantized buckets against the rate-fed
    sensor model (only meaningful with a noise-free camera). If `record`
    is given, the event words of every step are appended to it (a replay
    stream for sim/replay_bench.py).
    """
    agent = Agent()
    env = env if env is not None else build_default_environment()
    camera = camera if camera is not None else EventCamera()
    golden = GoldenPipeline() if check else None
    noise_free = camera.config.noise_rate == 0.0

    engine.reset()
    if golden is not None:
        golden.reset()

    log: list[dict] = []
    for step in range(max_steps):
        words = camera.sense(agent, env, step)
        if record is not None:
            record.extend(words)
        decision = engine.step(words)
        if check:
            check_decision(step, decision, golden.step(words), sense(agent, env) if noise_free else None)

        apply_action(agent, decision.state)
        log.append(
            {
                "step": step,
                "x": agent.x,
                "y": agent.y,
                "heading_deg": math.degrees(agent.heading),
                "ev_left": decision.ev_left,
                "ev_right": decision.ev_right,
                "state": decision.state,
                "action": ACTION_NAMES[decision.state],
                "brake_timer": decision.brake_timer,
                "events_left": decision.count_left,
                "events_right": decision.count_right,
                "events_sent": len(words),
            }
        )

        hit = check_collision(agent, env)
        if hit is not None:
            print(f"Collision at step {step}: agent=({agent.x:.2f}, {agent.y:.2f}) obstacle={hit}")
            break

    if log_path is not None:
        write_csv(log, log_path)
    return log, env


def write_csv(log: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(log[0].keys()))
        writer.writeheader()
        writer.writerows(log)
    print(f"Wrote {len(log)} rows to {path}")


def write_replay(words: list[int], log: list[dict], stem: Path) -> None:
    """Replay stream: all packets concatenated (TLAST only at the very end)
    plus the per-window decisions the hardware produced in closed loop."""
    stem.parent.mkdir(parents=True, exist_ok=True)
    bin_path = stem.with_suffix(".bin")
    bin_path.write_bytes(words_to_bytes(words))
    csv_path = stem.with_suffix(".expected.csv")
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["step", "ev_left", "ev_right", "state", "brake_timer"])
        writer.writeheader()
        for row in log:
            writer.writerow({k: row[k] for k in writer.fieldnames})
    print(f"Wrote replay stream {bin_path} ({len(words)} words, {len(log)} windows) and {csv_path}")


def plot_trajectory(log: list[dict], env: Environment, path: Path, title_suffix: str = "") -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    xs = [row["x"] for row in log]
    ys = [row["y"] for row in log]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(xs, ys, "-o", markersize=2, linewidth=1, label="agent path")

    for obs in env.obstacles:
        ax.add_patch(plt.Circle((obs.x, obs.y), obs.radius, color="tab:red", alpha=0.4))

    brake_xs = [row["x"] for row in log if row["action"] == "BRAKE"]
    brake_ys = [row["y"] for row in log if row["action"] == "BRAKE"]
    if brake_xs:
        ax.scatter(brake_xs, brake_ys, color="black", marker="x", label="BRAKE", zorder=5)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Neuromorphic FSM controller: agent trajectory" + title_suffix)
    ax.legend()
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"Saved trajectory plot to {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_engine_args(parser)
    parser.add_argument("--max-steps", type=int, default=45)
    parser.add_argument("--noise", type=float, default=0.0,
                        help="mean background DVS events per hemisphere per window (default 0)")
    parser.add_argument("--camera-seed", type=int, default=0)
    parser.add_argument("--record", type=Path, default=None,
                        help="also write a replay stream (<path>.bin + <path>.expected.csv)")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "results")
    args = parser.parse_args()

    camera = EventCamera(EventCameraConfig(noise_rate=args.noise), seed=args.camera_seed)
    record: list[int] | None = [] if args.record is not None else None

    engine = engine_from_args(args)
    try:
        print(f"Engine: {engine.name}")
        log, env = run_episode(
            engine,
            max_steps=args.max_steps,
            log_path=args.out_dir / "trajectory_log.csv",
            camera=camera,
            check=not args.no_check,
            record=record,
        )
    finally:
        engine.close()

    plot_trajectory(log, env, args.out_dir / "trajectory.png", f" ({engine.name})")
    if record is not None:
        write_replay(record, log, args.record)
    total_events = sum(row["events_sent"] for row in log)
    print(f"Episode length: {len(log)} steps, {total_events} event words streamed")
    if not args.no_check:
        print("Every decision matched the golden model" + (" and the rate-fed sensor model" if args.noise == 0 else ""))


if __name__ == "__main__":
    main()
