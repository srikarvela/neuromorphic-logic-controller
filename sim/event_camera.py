"""DVS-style event camera model.

Turns the per-hemisphere looming readings from sim/agent_sim.py into a
stream of address-event words (x, y, polarity, timestamp) on a 128x128
sensor, one control window per call. The hardware never sees a "rate":
it counts these events per hemisphere per window and quantizes the count.

Signal events: each hemisphere emits `intensity_to_count(intensity)` events
(floor(intensity * 100), so a looming obstacle at intensity 0.85 yields 85
events -- exactly the count that quantizes to bucket 3, keeping the
event-stream pipeline decision-for-decision identical to the old rate-fed
model). They are scattered around the dominant obstacle's bearing with a
spread matching its apparent angular size, random polarity, and timestamps
spread across the window in increasing order.

Noise events: with `noise_rate > 0`, a Poisson-distributed number of
background events per hemisphere per window lands at uniformly random
pixels. This deliberately breaks bit-exact parity with the rate-fed model
and is how the sweep probes the controller's robustness to sensor noise.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

from agent_sim import Agent, Environment, HemisphereReading, sense_readings
from event_stream import (
    SENSOR_HEIGHT,
    SENSOR_WIDTH,
    WINDOW_TICKS,
    X_CENTER,
    intensity_to_count,
    pack_event,
    pack_sync,
    window_base,
)


@dataclass
class EventCameraConfig:
    noise_rate: float = 0.0  # mean background events per hemisphere per window
    min_spread_px: float = 2.0  # never collapse a stimulus onto a single column


class EventCamera:
    def __init__(self, config: EventCameraConfig | None = None, seed: int = 0):
        self.config = config or EventCameraConfig()
        self.rng = random.Random(seed)

    def sense(self, agent: Agent, env: Environment, step: int) -> list[int]:
        """Event words for control step `step` (one window). Never empty."""
        left, right = sense_readings(agent, env)
        half_fov = env.fov / 2.0
        events: list[tuple[float, int, int, int]] = []  # (phase, x, y, pol)

        events += self._signal_events(left, half_fov, hemisphere="left")
        events += self._signal_events(right, half_fov, hemisphere="right")
        if self.config.noise_rate > 0.0:
            events += self._noise_events(hemisphere="left")
            events += self._noise_events(hemisphere="right")

        base = window_base(step)
        if not events:
            return [pack_sync(base)]

        events.sort(key=lambda e: e[0])
        return [pack_event(x, y, pol, base + int(phase * WINDOW_TICKS)) for phase, x, y, pol in events]

    # -- internals ----------------------------------------------------------
    def _x_range(self, hemisphere: str) -> tuple[int, int]:
        return (0, X_CENTER - 1) if hemisphere == "left" else (X_CENTER, SENSOR_WIDTH - 1)

    def _signal_events(self, reading: HemisphereReading, half_fov: float, hemisphere: str):
        n = intensity_to_count(reading.intensity)
        if n == 0:
            return []
        lo, hi = self._x_range(hemisphere)
        # bearing +half_fov (far left) -> x=0, 0 -> x=64, -half_fov -> x=127
        x_c = X_CENTER - (reading.bearing / half_fov) * X_CENTER
        spread = max((reading.angular_radius / half_fov) * X_CENTER, self.config.min_spread_px)
        y_c = SENSOR_HEIGHT / 2.0
        out = []
        for _ in range(n):
            x = int(round(x_c + self.rng.uniform(-spread, spread)))
            y = int(round(y_c + self.rng.uniform(-spread, spread)))
            out.append((self.rng.random(), min(max(x, lo), hi), min(max(y, 0), SENSOR_HEIGHT - 1),
                        1 if self.rng.random() < 0.5 else 0))
        return out

    def _noise_events(self, hemisphere: str):
        lo, hi = self._x_range(hemisphere)
        n = _poisson(self.rng, self.config.noise_rate)
        return [
            (self.rng.random(), self.rng.randint(lo, hi), self.rng.randint(0, SENSOR_HEIGHT - 1),
             1 if self.rng.random() < 0.5 else 0)
            for _ in range(n)
        ]


def _poisson(rng: random.Random, lam: float) -> int:
    # Knuth's algorithm; fine for the small rates used here.
    limit = math.exp(-lam)
    k, p = 0, 1.0
    while True:
        p *= rng.random()
        if p <= limit:
            return k
        k += 1
