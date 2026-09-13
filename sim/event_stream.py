"""Wire format shared by the RTL, the Icarus cosim server and the PYNQ DMA driver.

Every constant here has a twin in rtl/event_rate_window.sv / rtl/nlc_axis_top.sv;
sim/test_event_stream.py and the RTL testbenches keep the two in step.

Event word (32 bits, one AXI-Stream beat)::

    EVENT  bit31=1  [30] polarity  [29:23] y  [22:16] x  [15:0] timestamp
    CTRL   bit31=0  [30] reset     [29:16] 0            [15:0] timestamp

A CTRL word with reset=0 is a sync/heartbeat: it carries a timestamp (so it
can open or close a window) but counts nothing. A CTRL word with reset=1
returns the FSM to FORWARD and clears the open window.

Decision word (32 bits, one per closed window)::

    [31:24] window id   [23:22] ev_left  [21:20] ev_right  [19:18] state
    [17:15] brake timer [14] stale       [13:7] count_left [6:0] count_right
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

# Sensor geometry: 128 x 128 px, DVS128-like. x < X_CENTER is the left hemisphere.
SENSOR_WIDTH = 128
SENSOR_HEIGHT = 128
X_CENTER = 64

TS_BITS = 16
WINDOW_SHIFT = 8
WINDOW_TICKS = 1 << WINDOW_SHIFT  # timestamp ticks per control window
WINDOW_ID_MASK = (1 << (TS_BITS - WINDOW_SHIFT)) - 1  # 8-bit window ids

# Per-hemisphere saturating counter and the count -> bucket thresholds.
COUNT_MAX = 127
RATE_THRESHOLDS = (25, 55, 85)

# Events emitted per hemisphere per window at looming intensity 1.0. Chosen
# with the thresholds above so that quantize(intensity_to_count(I)) equals
# agent_sim._bucket(I): the event pipeline reproduces the rate-fed FSM's
# decisions exactly (checked in sim/test_event_stream.py).
MAX_EVENTS_PER_HEMISPHERE = 100

_EVENT_FLAG = 1 << 31
_POLARITY_FLAG = 1 << 30
_RESET_FLAG = 1 << 30


def pack_event(x: int, y: int, polarity: int, ts: int) -> int:
    if not (0 <= x < SENSOR_WIDTH and 0 <= y < SENSOR_HEIGHT):
        raise ValueError(f"pixel ({x}, {y}) off sensor")
    return (
        _EVENT_FLAG
        | (_POLARITY_FLAG if polarity else 0)
        | ((y & 0x7F) << 23)
        | ((x & 0x7F) << 16)
        | (ts & 0xFFFF)
    )


def pack_sync(ts: int) -> int:
    return ts & 0xFFFF


def pack_reset(ts: int = 0) -> int:
    return _RESET_FLAG | (ts & 0xFFFF)


def is_event(word: int) -> bool:
    return bool(word & _EVENT_FLAG)


def is_reset(word: int) -> bool:
    return not (word & _EVENT_FLAG) and bool(word & _RESET_FLAG)


def event_x(word: int) -> int:
    return (word >> 16) & 0x7F


def event_ts(word: int) -> int:
    return word & 0xFFFF


def window_id_of(word: int) -> int:
    return (event_ts(word) >> WINDOW_SHIFT) & WINDOW_ID_MASK


def window_base(step: int) -> int:
    """Timestamp of the first tick of control step `step`'s window."""
    return (step & WINDOW_ID_MASK) << WINDOW_SHIFT


def quantize(count: int) -> int:
    t1, t2, t3 = RATE_THRESHOLDS
    if count >= t3:
        return 3
    if count >= t2:
        return 2
    if count >= t1:
        return 1
    return 0


def intensity_to_count(intensity: float) -> int:
    """Looming intensity in [0, 1] -> number of events this window."""
    return min(int(intensity * MAX_EVENTS_PER_HEMISPHERE), COUNT_MAX)


@dataclass(frozen=True)
class Decision:
    window_id: int
    ev_left: int
    ev_right: int
    state: int
    brake_timer: int
    stale: bool
    count_left: int
    count_right: int
    last: bool = True

    @property
    def word(self) -> int:
        return (
            ((self.window_id & 0xFF) << 24)
            | ((self.ev_left & 3) << 22)
            | ((self.ev_right & 3) << 20)
            | ((self.state & 3) << 18)
            | ((self.brake_timer & 7) << 15)
            | ((1 if self.stale else 0) << 14)
            | ((self.count_left & 0x7F) << 7)
            | (self.count_right & 0x7F)
        )


def unpack_decision(word: int, last: bool = True) -> Decision:
    return Decision(
        window_id=(word >> 24) & 0xFF,
        ev_left=(word >> 22) & 3,
        ev_right=(word >> 20) & 3,
        state=(word >> 18) & 3,
        brake_timer=(word >> 15) & 7,
        stale=bool((word >> 14) & 1),
        count_left=(word >> 7) & 0x7F,
        count_right=word & 0x7F,
        last=last,
    )


def words_to_bytes(words: list[int]) -> bytes:
    """Little-endian u32 packing, the layout a 32-bit AXI DMA moves."""
    return struct.pack(f"<{len(words)}I", *words)


def bytes_to_words(data: bytes) -> list[int]:
    return list(struct.unpack(f"<{len(data) // 4}I", data[: len(data) // 4 * 4]))
