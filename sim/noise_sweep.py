"""Sensor-noise robustness sweep.

For each background-noise level (mean Poisson events per hemisphere per
window), run the randomized obstacle courses from sim/stress_test.py with
the hardware engine in the loop and measure:

- bucket_error_rate   fraction of windows where the hardware's quantized
                      rate bucket differs from the noise-free sensor model's
                      bucket at the same agent pose
- false_turn_rate     fraction of windows spent turning although the
                      noise-free sensor sees no obstacle on either side
- false_brake_rate    same, for windows spent in BRAKE
- collision_rate      fraction of episodes ending in a collision

Every decision is still cross-checked against the golden model, so the
curve describes the controller, not a simulation bug.

    python3 sim/noise_sweep.py --episodes 100 --levels 0,10,20,30,40,50,60,80,100
"""
from __future__ import annotations

import argparse
import csv
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from agent_sim import Agent, apply_action, check_collision, sense
from cosim_driver import check_decision
from engines import add_engine_args, engine_from_args
from event_camera import EventCamera, EventCameraConfig
from golden_model import GoldenPipeline
from stress_test import random_environment

ROOT = Path(__file__).resolve().parent.parent


def run_level(engine, noise: float, episodes: int, max_steps: int, seed: int, check: bool) -> dict:
    windows = bucket_errors = false_turns = false_brakes = clear_windows = collisions = 0
    for i in range(episodes):
        ep_seed = seed + i
        env = random_environment(random.Random(ep_seed))
        camera = EventCamera(EventCameraConfig(noise_rate=noise), seed=ep_seed)
        golden = GoldenPipeline() if check else None
        agent = Agent()
        engine.reset()
        if golden is not None:
            golden.reset()
        for step in range(max_steps):
            ref_left, ref_right = sense(agent, env)
            words = camera.sense(agent, env, step)
            d = engine.step(words)
            if golden is not None:
                check_decision(step, d, golden.step(words), None)
            windows += 1
            bucket_errors += (d.ev_left != ref_left) + (d.ev_right != ref_right)
            if ref_left < 2 and ref_right < 2:
                clear_windows += 1
                false_turns += d.state in (1, 2)
                false_brakes += d.state == 3
            apply_action(agent, d.state)
            if check_collision(agent, env) is not None:
                collisions += 1
                break
    return {
        "noise": noise,
        "episodes": episodes,
        "windows": windows,
        "bucket_error_rate": round(bucket_errors / (2 * windows), 4),
        "false_turn_rate": round(false_turns / max(clear_windows, 1), 4),
        "false_brake_rate": round(false_brakes / max(clear_windows, 1), 4),
        "collision_rate": round(collisions / episodes, 4),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_engine_args(parser)
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--max-steps", type=int, default=60)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--levels", default="0,10,20,30,40,50,60,70,80,90,100")
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "noise_sweep.csv")
    args = parser.parse_args()

    levels = [float(x) for x in args.levels.split(",")]
    engine = engine_from_args(args)
    print(f"Engine: {engine.name}")
    rows = []
    try:
        for noise in levels:
            row = run_level(engine, noise, args.episodes, args.max_steps, args.seed, not args.no_check)
            rows.append(row)
            print(f"noise={noise:5.1f}  bucket_err={row['bucket_error_rate']:.3f}  "
                  f"false_turn={row['false_turn_rate']:.3f}  false_brake={row['false_brake_rate']:.3f}  collisions={row['collision_rate']:.2f}")
    finally:
        engine.close()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
