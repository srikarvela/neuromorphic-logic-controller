"""Randomized multi-episode verification sweep.

Runs many independently-seeded random obstacle courses through the real
compiled FSM (via cosim_driver.run_fsm_step, i.e. actual vvp invocations,
not a software re-implementation of the controller) and reports the
collision rate. Each episode is reproducible from its seed alone, so a
failing run can be replayed with --episodes 1 --seed <n>.
"""
from __future__ import annotations

import argparse
import csv
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from agent_sim import ACTION_NAMES, Agent, Environment, Obstacle, apply_action, check_collision, sense
from cosim_driver import run_fsm_step

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


def run_single_episode(env: Environment, max_steps: int) -> dict:
    agent = Agent()
    state, brake_timer = 0, 0
    brake_engagements = 0
    turn_engagements = 0
    prev_state = 0

    for step in range(max_steps):
        ev_left, ev_right = sense(agent, env)
        state, brake_timer = run_fsm_step(state, brake_timer, ev_left, ev_right)
        apply_action(agent, state)

        if state != prev_state:
            if state == 3:
                brake_engagements += 1
            elif state in (1, 2):
                turn_engagements += 1
        prev_state = state

        hit = check_collision(agent, env)
        if hit is not None:
            return {
                "collision": True,
                "steps": step + 1,
                "turn_engagements": turn_engagements,
                "brake_engagements": brake_engagements,
                "final_x": round(agent.x, 2),
                "final_y": round(agent.y, 2),
            }

    return {
        "collision": False,
        "steps": max_steps,
        "turn_engagements": turn_engagements,
        "brake_engagements": brake_engagements,
        "final_x": round(agent.x, 2),
        "final_y": round(agent.y, 2),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=25)
    parser.add_argument("--max-steps", type=int, default=60)
    parser.add_argument("--seed", type=int, default=0, help="base seed; episode i uses seed+i")
    parser.add_argument(
        "--out", type=Path, default=ROOT / "results" / "stress_test_summary.csv"
    )
    args = parser.parse_args()

    rows = []
    collisions = 0
    for i in range(args.episodes):
        episode_seed = args.seed + i
        rng = random.Random(episode_seed)
        env = random_environment(rng)
        result = run_single_episode(env, args.max_steps)
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
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
