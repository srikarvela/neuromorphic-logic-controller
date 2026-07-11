"""Hardware-in-the-loop driver.

Python owns the agent/environment physics; the FSM "brain" runs as real
compiled SystemVerilog, invoked once per control step via `vvp`. See
docs/architecture.md for how state is carried between invocations.
"""
from __future__ import annotations

import csv
import math
import re
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from agent_sim import (
    ACTION_NAMES,
    Agent,
    Environment,
    Obstacle,
    apply_action,
    check_collision,
    sense,
)

ROOT = Path(__file__).resolve().parent.parent
VVP_BIN = ROOT / "build" / "tb_cosim_step.vvp"

RESULT_RE = re.compile(
    r"RESULT state_out=(\d+) brake_timer_out=(\d+) "
    r"cmd_forward=(\d) cmd_turn_left=(\d) cmd_turn_right=(\d) cmd_brake=(\d)"
)


def run_fsm_step(state: int, brake_timer: int, ev_left: int, ev_right: int) -> tuple[int, int]:
    """Invoke the compiled SystemVerilog FSM for exactly one control step."""
    if not VVP_BIN.exists():
        raise FileNotFoundError(f"{VVP_BIN} not found -- run scripts/build_cosim.sh first")

    proc = subprocess.run(
        [
            "vvp",
            str(VVP_BIN),
            f"+state_in={state}",
            f"+brake_timer_in={brake_timer}",
            f"+ev_left={ev_left}",
            f"+ev_right={ev_right}",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    for line in proc.stdout.splitlines():
        match = RESULT_RE.search(line)
        if match:
            return int(match.group(1)), int(match.group(2))
    raise RuntimeError(f"no RESULT line in vvp output:\n{proc.stdout}\n{proc.stderr}")


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


def run_episode(max_steps: int = 200, log_path: Path | None = None):
    agent = Agent()
    env = build_default_environment()

    state = 0
    brake_timer = 0
    log: list[dict] = []

    for step in range(max_steps):
        ev_left, ev_right = sense(agent, env)
        state, brake_timer = run_fsm_step(state, brake_timer, ev_left, ev_right)
        apply_action(agent, state)

        log.append(
            {
                "step": step,
                "x": agent.x,
                "y": agent.y,
                "heading_deg": math.degrees(agent.heading),
                "ev_left": ev_left,
                "ev_right": ev_right,
                "state": state,
                "action": ACTION_NAMES[state],
                "brake_timer": brake_timer,
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


def plot_trajectory(log: list[dict], env: Environment, path: Path) -> None:
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
    ax.set_title("Neuromorphic FSM controller: agent trajectory")
    ax.legend()
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"Saved trajectory plot to {path}")


def main() -> None:
    log, env = run_episode(max_steps=45, log_path=ROOT / "results" / "trajectory_log.csv")
    plot_trajectory(log, env, ROOT / "results" / "trajectory.png")
    print(f"Episode length: {len(log)} steps")


if __name__ == "__main__":
    main()
