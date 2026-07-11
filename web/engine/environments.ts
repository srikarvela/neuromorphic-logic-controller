import { makeEnvironment, type Environment, type Obstacle } from "./agentSim.ts";
import { mulberry32, randInt, uniform } from "./rng.ts";

/** Mirrors sim/cosim_driver.py::build_default_environment -- three obstacles staggered along the agent's own post-avoidance drift path so each one gets a distinct reaction. */
export function defaultEnvironment(): Environment {
  return makeEnvironment({
    obstacles: [
      { x: 10.0, y: 0.8, radius: 1.2 },
      { x: 18.7, y: -6.0, radius: 1.3 },
      { x: 24.7, y: -19.9, radius: 1.3 },
    ],
  });
}

/** Mirrors sim/stress_test.py::random_environment. */
export function randomEnvironment(seed: number): Environment {
  const rng = mulberry32(seed);
  const nObstacles = randInt(rng, 2, 5);
  const obstacles: Obstacle[] = [];
  let x = uniform(rng, 8.0, 14.0);
  for (let i = 0; i < nObstacles; i++) {
    const y = uniform(rng, -2.5, 2.5);
    const radius = uniform(rng, 0.8, 1.8);
    obstacles.push({ x, y, radius });
    x += uniform(rng, 6.0, 12.0);
  }
  return makeEnvironment({ obstacles });
}
