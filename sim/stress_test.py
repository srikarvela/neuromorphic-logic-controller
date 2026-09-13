"""Randomized multi-episode verification sweep.

Runs many independently-seeded random obstacle courses through the real
controller pipeline -- RTL under Icarus, or the FPGA on the PYNQ-Z2 -- and
reports the collision rate. Decisions always come from the hardware; the
Python golden model only cross-checks them. Each episode is reproducible
from its seed alone, so a failing run can be replayed with
--episodes 1 --seed <n>.

With --noise N the event camera adds Poisson background events, which is
the sweep's way of probing how the fixed thresholds hold up against sensor
noise (the rate-fed parity check is skipped in that mode, the golden-model
check is not).
"""
from __future__ import annotations

import argparse
import csv
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from agent_sim import Environment, Obstacle
from cosim_driver import run_episode
from engines import Engine, add_engine_args, engine_from_args
from event_camera import EventCamera, EventCameraConfig

ROOT = Path(__file__).resolve().parent.parent


def random_environment(rng: random.Random) -> Environment:
    # y is kept close to the agent's nominal straight-line path (y=0) so
    # obstacles are actually a threat worth reacting to -- a wide spread
    # mostly produces obstacles the agent was never going to hit anyway,
    # which defeats the point of a stress test.
    n_obstacles = rng.randint(2, 5)
    obstacles = []
    x = rng.uniform(8.0, 14.0)
    for _ in range(n_obstacles):
        y = rng.uniform(-2.5, 2.5)
        radius = rng.uniform(0.8, 1.8)
        obstacles.append(Obstacle(x=x, y=y, radius=radius))
        x += rng.uniform(6.0, 12.0)
    return Environment(obstacles=obstacles)


def run_single_episode(engine: Engine, env: Environment, max_steps: int, camera: EventCamera,
                       check: bool = True) -> dict:
    log, _ = run_episode(engine, env=env, max_steps=max_steps, camera=camera, check=check)

    brake_engagements = 0
    turn_engagements = 0
    prev_state = 0
    for row in log:
        state = row["state"]
        if state != prev_state:
            if state == 3:
                brake_engagements += 1
            elif state in (1, 2):
                turn_engagements += 1
        prev_state = state

    last = log[-1]
    return {
        "collision": len(log) < max_steps,
        "steps": len(log),
        "turn_engagements": turn_engagements,
        "brake_engagements": brake_engagements,
        "final_x": round(last["x"], 2),
        "final_y": round(last["y"], 2),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_engine_args(parser)
    parser.add_argument("--episodes", type=int, default=25)
    parser.add_argument("--max-steps", type=int, default=60)
    parser.add_argument("--seed", type=int, default=0, help="base seed; episode i uses seed+i")
    parser.add_argument("--noise", type=float, default=0.0,
                        help="mean background DVS events per hemisphere per window (default 0)")
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "stress_test_summary.csv")
    args = parser.parse_args()

    engine = engine_from_args(args)
    print(f"Engine: {engine.name}")
    rows = []
    collisions = 0
    try:
        for i in range(args.episodes):
            episode_seed = args.seed + i
            rng = random.Random(episode_seed)
            env = random_environment(rng)
            camera = EventCamera(EventCameraConfig(noise_rate=args.noise), seed=episode_seed)
            result = run_single_episode(engine, env, args.max_steps, camera, check=not args.no_check)
            result["seed"] = episode_seed
            result["n_obstacles"] = len(env.obstacles)
            rows.append(result)

            if result["collision"]:
                collisions += 1
                print(
                    f"[COLLISION] seed={episode_seed} step={result['steps']} "
                    f"n_obstacles={result['n_obstacles']} turns={result['turn_engagements']} "
                    f"brakes={result['brake_engagements']} final=({result['final_x']},{result['final_y']})"
                )
            else:
                print(
                    f"[OK]        seed={episode_seed} steps={result['steps']} "
                    f"n_obstacles={result['n_obstacles']} turns={result['turn_engagements']} "
                    f"brakes={result['brake_engagements']}"
                )
    finally:
        engine.close()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    collision_rate = collisions / args.episodes
    reacted = sum(1 for r in rows if r["turn_engagements"] or r["brake_engagements"])
    print("----------------------------------------")
    print(f"{args.episodes} episodes, {collisions} collisions ({collision_rate:.1%})")
    print(f"{reacted}/{args.episodes} episodes triggered at least one avoidance reaction")
    if not args.no_check:
        print("Every hardware decision matched the golden model")
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
