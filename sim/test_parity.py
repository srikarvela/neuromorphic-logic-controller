"""Parity of the event-stream pipeline (RTL under Icarus) with the original
rate-fed controller, and of the Icarus RTL with the golden model.

Needs `iverilog`/`vvp` on PATH; builds the cosim server on demand.
"""
from __future__ import annotations

import csv
import random
import shutil
import subprocess
from pathlib import Path

import pytest

from cosim_driver import build_default_environment, run_episode
from engines import VVP_BIN, IcarusEngine
from event_camera import EventCamera, EventCameraConfig
from event_stream import pack_event, pack_reset, pack_sync, window_base
from golden_model import GoldenPipeline
from stress_test import random_environment, run_single_episode
from test_event_stream import DIRECTED_VECTORS, window_words

ROOT = Path(__file__).resolve().parent.parent
GOLDEN = ROOT / "sim" / "golden"

pytestmark = pytest.mark.skipif(shutil.which("iverilog") is None or shutil.which("vvp") is None,
                                reason="Icarus Verilog not installed")


@pytest.fixture(scope="module", autouse=True)
def build_server():
    subprocess.run([str(ROOT / "scripts" / "build_cosim.sh")], check=True, capture_output=True)
    assert VVP_BIN.exists()


@pytest.fixture
def engine():
    eng = IcarusEngine()
    yield eng
    eng.close()


def _read_csv(path: Path) -> list[dict]:
    with path.open() as f:
        return list(csv.DictReader(f))


def test_directed_vectors_through_rtl(engine):
    assert engine.reset().state == 0
    for i, (el, er, expected) in enumerate(DIRECTED_VECTORS):
        d = engine.step(window_words(i, el, er))
        assert (d.window_id, d.ev_left, d.ev_right, d.state) == (i, el, er, expected), f"vector {i}"


def test_default_course_matches_ratefed_trajectory(engine):
    log, _ = run_episode(engine, max_steps=45, camera=EventCamera(), check=True)
    ref = _read_csv(GOLDEN / "trajectory_default_ratefed.csv")
    assert len(log) == len(ref)
    for row, exp in zip(log, ref):
        assert (row["ev_left"], row["ev_right"], row["state"], row["brake_timer"]) == \
               (int(exp["ev_left"]), int(exp["ev_right"]), int(exp["state"]), int(exp["brake_timer"])), row["step"]
        assert row["x"] == pytest.approx(float(exp["x"]), abs=1e-9)
        assert row["y"] == pytest.approx(float(exp["y"]), abs=1e-9)
        assert row["heading_deg"] == pytest.approx(float(exp["heading_deg"]), abs=1e-9)


def test_stress_sweep_matches_ratefed_summary(engine):
    ref = _read_csv(GOLDEN / "stress_seed0_25_ratefed.csv")
    for exp in ref:
        seed = int(exp["seed"])
        env = random_environment(random.Random(seed))
        got = run_single_episode(engine, env, max_steps=60, camera=EventCamera(seed=seed), check=True)
        assert got["collision"] == (exp["collision"] == "True"), seed
        assert (got["steps"], got["turn_engagements"], got["brake_engagements"]) == \
               (int(exp["steps"]), int(exp["turn_engagements"]), int(exp["brake_engagements"])), seed
        assert (got["final_x"], got["final_y"]) == (float(exp["final_x"]), float(exp["final_y"])), seed


def test_rtl_matches_golden_on_random_streams(engine):
    """Random windows incl. empty ones, gaps, stale events and mid-stream
    resets, streamed as multi-window packets: RTL == golden, decision for decision."""
    rng = random.Random(11)
    golden = GoldenPipeline()
    engine.reset()
    golden.reset()
    step = 0
    for _ in range(40):
        words: list[int] = []
        n_windows = rng.randint(1, 5)
        for _ in range(n_windows):
            base = window_base(step)
            n_left, n_right = rng.choice([0, 0, 10, 24, 25, 60, 90, 130]), rng.choice([0, 0, 10, 55, 85, 127])
            ev = [pack_event(rng.randrange(64), rng.randrange(128), rng.randrange(2), base + rng.randrange(256))
                  for _ in range(n_left)]
            ev += [pack_event(64 + rng.randrange(64), rng.randrange(128), rng.randrange(2), base + rng.randrange(256))
                   for _ in range(n_right)]
            ev.sort(key=lambda w: w & 0xFFFF)
            if rng.random() < 0.1 and ev:  # a stale straggler from the previous window
                ev.insert(rng.randrange(len(ev)), pack_event(3, 3, 0, window_base(step - 1) + 200))
            if rng.random() < 0.1:  # skip a window entirely
                step += 1
            words += ev or [pack_sync(base)]
            step += 1
        if rng.random() < 0.15:
            words = [pack_reset(window_base(step))]
            step += 1
        hw = engine.stream(words)
        ref = golden.stream(words)
        assert hw == ref, f"packet {_} diverged"


def test_noisy_camera_still_matches_golden(engine):
    env = build_default_environment()
    log, _ = run_episode(engine, env=env, max_steps=30,
                         camera=EventCamera(EventCameraConfig(noise_rate=8.0), seed=2), check=True)
    assert len(log) > 0
