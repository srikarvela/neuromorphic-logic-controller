"""Wire-format and model-consistency tests (no simulator needed)."""
from __future__ import annotations

import random

import pytest

from agent_sim import Agent, Environment, Obstacle, _bucket, sense
from event_camera import EventCamera, EventCameraConfig
from event_stream import (
    COUNT_MAX,
    MAX_EVENTS_PER_HEMISPHERE,
    RATE_THRESHOLDS,
    WINDOW_TICKS,
    Decision,
    bytes_to_words,
    event_ts,
    event_x,
    intensity_to_count,
    is_event,
    is_reset,
    pack_event,
    pack_reset,
    pack_sync,
    quantize,
    unpack_decision,
    window_base,
    window_id_of,
    words_to_bytes,
)
from golden_model import GoldenPipeline, fsm_step

# {ev_left, ev_right, expected state}: the same directed sequence as
# tb/tb_fsm_controller.sv and tb/tb_nlc_axis_top.sv scenario A.
DIRECTED_VECTORS = [
    (0, 0, 0), (3, 0, 2), (3, 0, 2), (0, 0, 0), (0, 3, 1), (0, 0, 0), (3, 3, 3),
    (0, 0, 3), (0, 0, 3), (0, 0, 3), (0, 0, 3), (0, 0, 0),
    (3, 0, 2), (3, 3, 3), (0, 0, 3), (0, 0, 3), (0, 0, 3), (0, 0, 3), (0, 0, 0),
    (0, 3, 1), (3, 3, 3), (0, 0, 3), (0, 0, 3), (0, 0, 3), (0, 0, 3), (0, 0, 0),
    (3, 3, 3), (3, 3, 3), (3, 3, 3), (3, 3, 3), (3, 3, 3), (3, 3, 0), (3, 3, 3),
]

BUCKET_COUNTS = {0: 0, 1: 30, 2: 60, 3: 100}


def window_words(step: int, ev_left: int, ev_right: int) -> list[int]:
    base = window_base(step)
    words = [pack_event(5 + i % 50, 40, i % 2, base + i) for i in range(BUCKET_COUNTS[ev_left])]
    words += [pack_event(70 + i % 50, 40, i % 2, base + 128 + i % 128) for i in range(BUCKET_COUNTS[ev_right])]
    return words or [pack_sync(base)]


def test_event_word_roundtrip():
    w = pack_event(x=37, y=100, polarity=1, ts=0x1234)
    assert is_event(w) and not is_reset(w)
    assert event_x(w) == 37 and event_ts(w) == 0x1234
    assert window_id_of(w) == 0x12
    assert w >> 30 == 0b11
    assert (w >> 23) & 0x7F == 100


def test_ctrl_words():
    assert not is_event(pack_sync(0x0100)) and not is_reset(pack_sync(0x0100))
    assert is_reset(pack_reset(0x0100)) and not is_event(pack_reset(0x0100))
    assert window_id_of(pack_sync(0x0100)) == 1


def test_event_word_rejects_off_sensor():
    with pytest.raises(ValueError):
        pack_event(128, 0, 0, 0)


def test_decision_word_roundtrip():
    d = Decision(window_id=0xAB, ev_left=3, ev_right=1, state=2, brake_timer=4, stale=True,
                 count_left=127, count_right=9, last=False)
    assert unpack_decision(d.word, last=False) == d


def test_bytes_roundtrip_little_endian():
    words = [pack_event(1, 2, 0, 3), pack_sync(0x100), 0xDEADBEEF]
    data = words_to_bytes(words)
    assert data[:4] == bytes([0x03, 0x00, 0x01, 0x81])
    assert bytes_to_words(data) == words


def test_quantize_thresholds():
    t1, t2, t3 = RATE_THRESHOLDS
    assert [quantize(c) for c in (0, t1 - 1, t1, t2 - 1, t2, t3 - 1, t3, COUNT_MAX)] == [0, 0, 1, 1, 2, 2, 3, 3]


def test_count_quantization_matches_rate_model():
    """The whole parity argument: floor(I*100) through the hardware thresholds
    gives the same bucket as the software model's intensity thresholds."""
    for i in range(0, 100001):
        intensity = i / 100000
        assert quantize(intensity_to_count(intensity)) == _bucket(intensity), intensity
    for boundary in (0.25, 0.55, 0.85):
        assert quantize(intensity_to_count(boundary)) == _bucket(boundary)
    assert intensity_to_count(1.0) == MAX_EVENTS_PER_HEMISPHERE <= COUNT_MAX


def test_fsm_port_matches_unit_testbench_vectors():
    state, timer = 0, 0
    for i, (el, er, expected) in enumerate(DIRECTED_VECTORS):
        state, timer = fsm_step(state, timer, el, er)
        assert state == expected, f"vector {i}"


def test_golden_closed_loop_matches_vectors():
    g = GoldenPipeline()
    assert g.reset().state == 0
    for i, (el, er, expected) in enumerate(DIRECTED_VECTORS):
        d = g.step(window_words(i, el, er))
        assert (d.window_id, d.ev_left, d.ev_right, d.state) == (i, el, er, expected), f"vector {i}"
        assert (d.count_left, d.count_right) == (BUCKET_COUNTS[el], BUCKET_COUNTS[er])
        assert d.last and not d.stale


def test_golden_replay_equals_closed_loop():
    vectors = DIRECTED_VECTORS
    closed = GoldenPipeline()
    closed.reset()
    closed_decisions = [closed.step(window_words(i, el, er)) for i, (el, er, _) in enumerate(vectors)]

    replay = GoldenPipeline()
    replay.reset()
    words = [w for i, (el, er, _) in enumerate(vectors) for w in window_words(i, el, er)]
    replay_decisions = replay.stream(words)

    assert len(replay_decisions) == len(vectors)
    assert [d.last for d in replay_decisions] == [False] * (len(vectors) - 1) + [True]
    for a, b in zip(closed_decisions, replay_decisions):
        assert (a.window_id, a.ev_left, a.ev_right, a.state, a.brake_timer, a.count_left, a.count_right) == \
               (b.window_id, b.ev_left, b.ev_right, b.state, b.brake_timer, b.count_left, b.count_right)


def test_golden_catch_up_stale_and_saturation():
    g = GoldenPipeline()
    g.reset()
    # window 20: 5 left, 7 right; jump to 23 -> 20 closes, 21 and 22 close empty
    words = [pack_event(1, 1, 0, window_base(20) + i) for i in range(5)]
    words += [pack_event(100, 1, 0, window_base(20) + 10 + i) for i in range(7)]
    words += [pack_sync(window_base(23))]
    out = g.stream(words)
    assert [(d.window_id, d.count_left, d.count_right, d.last) for d in out] == [
        (20, 5, 7, False), (21, 0, 0, False), (22, 0, 0, False), (23, 0, 0, True)]
    # stale event (behind the open window) is dropped and flagged
    out = g.stream([pack_event(1, 1, 0, window_base(40)), pack_event(1, 1, 0, window_base(39)),
                    pack_event(100, 1, 0, window_base(40) + 5)])
    assert len(out) == 1 and out[0].stale and (out[0].count_left, out[0].count_right) == (1, 1)
    # saturating counter
    out = g.stream([pack_event(1, 1, 0, window_base(41) + (i % 256)) for i in range(200)])
    assert out[0].count_left == COUNT_MAX and out[0].ev_left == 3


def test_golden_reset_reports_reset_window():
    g = GoldenPipeline()
    g.step(window_words(0, 3, 3))
    assert g.state == 3
    d = g.step([pack_reset(window_base(9))])
    assert (d.window_id, d.state, d.brake_timer, d.count_left, d.count_right) == (9, 0, 0, 0, 0)


def _two_obstacle_env() -> tuple[Agent, Environment]:
    env = Environment(obstacles=[Obstacle(x=6.0, y=1.0, radius=1.0), Obstacle(x=7.0, y=-2.5, radius=0.8)])
    return Agent(x=0.0, y=0.0), env


def test_camera_counts_reproduce_rate_buckets():
    agent, env = _two_obstacle_env()
    cam = EventCamera(seed=3)
    for step in range(5):
        agent.x += 0.7
        words = cam.sense(agent, env, step)
        left = sum(1 for w in words if is_event(w) and event_x(w) < 64)
        right = sum(1 for w in words if is_event(w) and event_x(w) >= 64)
        assert (quantize(left), quantize(right)) == sense(agent, env)
        assert all(window_id_of(w) == step for w in words)
        ts = [event_ts(w) for w in words]
        assert ts == sorted(ts) and all(window_base(step) <= t < window_base(step) + WINDOW_TICKS for t in ts)


def test_camera_empty_window_is_a_sync_word():
    cam = EventCamera()
    words = cam.sense(Agent(), Environment(obstacles=[]), step=7)
    assert words == [pack_sync(window_base(7))]


def test_camera_noise_adds_events_only():
    agent, env = _two_obstacle_env()
    clean = EventCamera(seed=1).sense(agent, env, 0)
    noisy = EventCamera(EventCameraConfig(noise_rate=20.0), seed=1).sense(agent, env, 0)
    assert len(noisy) > len(clean)
    assert all(is_event(w) for w in noisy)


def test_golden_is_deterministic_under_random_streams():
    rng = random.Random(5)
    a, b = GoldenPipeline(), GoldenPipeline()
    for step in range(200):
        words = window_words(step, rng.randrange(4), rng.randrange(4))
        assert a.step(words) == b.step(words)
