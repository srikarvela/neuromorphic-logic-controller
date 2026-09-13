"""Pure-Python reference model of the whole event pipeline (rtl/nlc_axis_top.sv).

This is NOT what produces the project's results -- the hardware-in-the-loop
harness always takes its decisions from the RTL (Icarus) or from the FPGA.
The golden model rides alongside as a cross-check: every decision the
hardware returns is compared against it, so a divergence between silicon,
simulation and the documented behaviour shows up on the step it happens.

It mirrors the RTL structure deliberately: a windowed counter with the same
window-id, catch-up, stale-drop and in-band-reset semantics, feeding a port
of fsm_controller's transition logic stepped once per closed window.
"""
from __future__ import annotations

from dataclasses import dataclass

from event_stream import (
    COUNT_MAX,
    WINDOW_ID_MASK,
    X_CENTER,
    Decision,
    event_x,
    is_event,
    is_reset,
    quantize,
    window_id_of,
)

FORWARD, TURN_L, TURN_R, BRAKE_S = 0, 1, 2, 3


@dataclass(frozen=True)
class FsmParams:
    obstacle_thresh: int = 2
    brake_thresh: int = 3
    brake_cycles: int = 4


DEFAULT_FSM_PARAMS = FsmParams()


def fsm_step(state: int, brake_timer: int, ev_left: int, ev_right: int,
             params: FsmParams = DEFAULT_FSM_PARAMS) -> tuple[int, int]:
    """One control step of rtl/fsm_controller.sv: always_comb then always_ff."""
    obstacle_left = ev_left >= params.obstacle_thresh
    obstacle_right = ev_right >= params.obstacle_thresh
    critical_front = ev_left >= params.brake_thresh and ev_right >= params.brake_thresh

    next_state = state
    timer_next = brake_timer
    if state == FORWARD:
        if critical_front:
            next_state = BRAKE_S
        elif obstacle_left:
            next_state = TURN_R
        elif obstacle_right:
            next_state = TURN_L
    elif state == TURN_R:
        if critical_front:
            next_state = BRAKE_S
        elif not obstacle_left:
            next_state = FORWARD
    elif state == TURN_L:
        if critical_front:
            next_state = BRAKE_S
        elif not obstacle_right:
            next_state = FORWARD
    elif state == BRAKE_S:
        if brake_timer == 0:
            next_state = FORWARD
        else:
            timer_next = brake_timer - 1

    if next_state == BRAKE_S and state != BRAKE_S:
        timer_next = params.brake_cycles
    return next_state, timer_next


class GoldenPipeline:
    """Same packet-in / decisions-out contract as the hardware engines."""

    name = "python golden model"

    def __init__(self, params: FsmParams = DEFAULT_FSM_PARAMS):
        self.params = params
        self.state = FORWARD
        self.brake_timer = 0
        self.win_open = False
        self.cur_id = 0
        self.count_left = 0
        self.count_right = 0
        self.stale = False

    # -- engine interface ---------------------------------------------------
    def reset(self) -> Decision:
        from event_stream import pack_reset
        return self.step([pack_reset(0)])

    def step(self, words: list[int]) -> Decision:
        decisions = self.stream(words)
        if len(decisions) != 1:
            raise RuntimeError(
                f"closed-loop packet produced {len(decisions)} decisions, expected 1 "
                "(non-consecutive window ids in the stream?)"
            )
        return decisions[0]

    def stream(self, words: list[int]) -> list[Decision]:
        out: list[Decision] = []
        for i, w in enumerate(words):
            out.extend(self._process(w, last=(i == len(words) - 1)))
        return out

    def close(self) -> None:
        pass

    # -- RTL mirror ---------------------------------------------------------
    def _close_window(self, wid: int, last: bool, is_reset_close: bool = False) -> Decision:
        if is_reset_close:
            self.state, self.brake_timer = FORWARD, 0
            cl = cr = 0
            stale = False
        else:
            cl, cr = self.count_left, self.count_right
            stale = self.stale
            self.state, self.brake_timer = fsm_step(
                self.state, self.brake_timer, quantize(cl), quantize(cr), self.params
            )
        self.count_left = self.count_right = 0
        self.stale = False
        return Decision(
            window_id=wid & 0xFF,
            ev_left=quantize(cl),
            ev_right=quantize(cr),
            state=self.state,
            brake_timer=self.brake_timer,
            stale=stale,
            count_left=cl,
            count_right=cr,
            last=last,
        )

    def _process(self, word: int, last: bool) -> list[Decision]:
        out: list[Decision] = []
        wid = window_id_of(word)

        if is_reset(word):
            self.win_open = False
            out.append(self._close_window(wid, last, is_reset_close=True))
            return out

        stale = False
        if self.win_open and wid != self.cur_id:
            delta = (wid - self.cur_id) & WINDOW_ID_MASK
            if delta >= (WINDOW_ID_MASK + 1) // 2:
                stale = True
            else:
                while self.cur_id != wid:
                    out.append(self._close_window(self.cur_id, last=False))
                    self.cur_id = (self.cur_id + 1) & WINDOW_ID_MASK

        if not self.win_open:
            self.win_open = True
            self.cur_id = wid

        if stale:
            self.stale = True
        elif is_event(word):
            if event_x(word) < X_CENTER:
                self.count_left = min(self.count_left + 1, COUNT_MAX)
            else:
                self.count_right = min(self.count_right + 1, COUNT_MAX)

        if last:
            out.append(self._close_window(self.cur_id, last=True))
            self.win_open = False
        return out
