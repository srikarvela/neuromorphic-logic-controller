"""2D agent + environment used to drive the neuromorphic FSM controller.

The "sensor" here is a coarse stand-in for an event camera: instead of
simulating individual pixel-level brightness-change events, we compute a
looming proxy per hemisphere (left/right half of the field of view) from
obstacle distance and bearing, and quantize it into the same 0-3 event-rate
buckets the FSM expects on ev_left/ev_right.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

ACTION_FORWARD = 0
ACTION_TURN_L = 1
ACTION_TURN_R = 2
ACTION_BRAKE = 3

ACTION_NAMES = {
    ACTION_FORWARD: "FORWARD",
    ACTION_TURN_L: "TURN_L",
    ACTION_TURN_R: "TURN_R",
    ACTION_BRAKE: "BRAKE",
}


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float


@dataclass
class Agent:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians, 0 = +x axis, positive = counterclockwise (left)
    speed: float = 1.0  # units/step while moving forward
    turn_rate: float = math.radians(15)  # radians/step while turning
    radius: float = 0.3  # collision radius


@dataclass
class Environment:
    obstacles: list[Obstacle] = field(default_factory=list)
    fov: float = math.radians(90)  # total field of view
    sense_range: float = 12.0


def sense(agent: Agent, env: Environment) -> tuple[int, int]:
    """Return (ev_left, ev_right) event-rate buckets in [0, 3].

    For each obstacle within the sensor's field of view and range, compute
    a looming intensity that grows as the obstacle gets closer, then bucket
    the strongest reading on each hemisphere into 0..3. Positive bearing
    (counterclockwise from heading) is the left hemisphere.
    """
    half_fov = env.fov / 2.0
    left_intensity = 0.0
    right_intensity = 0.0

    for obs in env.obstacles:
        dx = obs.x - agent.x
        dy = obs.y - agent.y
        dist = max(math.hypot(dx, dy) - obs.radius - agent.radius, 0.0)
        if dist > env.sense_range:
            continue

        bearing = _wrap_angle(math.atan2(dy, dx) - agent.heading)
        if abs(bearing) > half_fov:
            continue

        proximity = 1.0 - (dist / env.sense_range)  # 0..1, closer = higher
        intensity = proximity ** 2

        if bearing >= 0:
            left_intensity = max(left_intensity, intensity)
        else:
            right_intensity = max(right_intensity, intensity)

    return _bucket(left_intensity), _bucket(right_intensity)


def _bucket(intensity: float) -> int:
    if intensity >= 0.85:
        return 3
    if intensity >= 0.55:
        return 2
    if intensity >= 0.25:
        return 1
    return 0


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def apply_action(agent: Agent, action: int) -> None:
    if action == ACTION_FORWARD:
        agent.x += agent.speed * math.cos(agent.heading)
        agent.y += agent.speed * math.sin(agent.heading)
    elif action == ACTION_TURN_L:
        agent.heading = _wrap_angle(agent.heading + agent.turn_rate)
        agent.x += 0.5 * agent.speed * math.cos(agent.heading)
        agent.y += 0.5 * agent.speed * math.sin(agent.heading)
    elif action == ACTION_TURN_R:
        agent.heading = _wrap_angle(agent.heading - agent.turn_rate)
        agent.x += 0.5 * agent.speed * math.cos(agent.heading)
        agent.y += 0.5 * agent.speed * math.sin(agent.heading)
    elif action == ACTION_BRAKE:
        pass  # hold position
    else:
        raise ValueError(f"unknown action {action}")


def check_collision(agent: Agent, env: Environment) -> Obstacle | None:
    for obs in env.obstacles:
        if math.hypot(obs.x - agent.x, obs.y - agent.y) <= obs.radius + agent.radius:
            return obs
    return None
