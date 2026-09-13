"""PYNQ-Z2 engine: the event pipeline on real silicon, driven over AXI DMA.

Same contract as sim/engines.py::IcarusEngine, so sim/cosim_driver.py,
sim/stress_test.py and sim/replay_bench.py run unchanged against the FPGA
with `--engine pynq`. Same conventions as the crypto feed handler's
pynq_driver.py (Overlay + allocate + sendchannel/recvchannel, dry-run
when the pynq package is missing).

Per control step:
    tx[0:n] <- event words        dma.recvchannel.transfer(rx[0:1])
                                  dma.sendchannel.transfer(tx[0:n])
                                  wait for both
    Decision <- rx[0]

The bitstream is built by `make bitstream` (fpga/tcl/build_bitstream.tcl);
copy nlc.bit + nlc.hwh next to this file on the board (`make board-deploy`).

Run on the board from the repo root, e.g.:
    python3 sim/cosim_driver.py --engine pynq
    python3 sim/stress_test.py --engine pynq --episodes 25
    python3 sim/replay_bench.py --engine pynq --repeat 20
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "sim"))
from event_stream import Decision, pack_reset, unpack_decision  # noqa: E402

try:
    import numpy as np
    from pynq import Overlay, allocate
    PYNQ_AVAILABLE = True
except ImportError:  # pragma: no cover - only on the board
    PYNQ_AVAILABLE = False

DEFAULT_BITSTREAM = Path(__file__).resolve().parent / "nlc.bit"
MAX_TX_WORDS = 1 << 16   # replay streams: up to 65536 words per DMA packet
MAX_RX_WORDS = 1 << 12   # decisions per packet (one per window)


class PynqEngine:
    name = "RTL on PYNQ-Z2 (XC7Z020) over AXI DMA"

    def __init__(self, bitstream: Path = DEFAULT_BITSTREAM, timeout_s: float = 2.0, dma_name: str = "axi_dma_0"):
        if not PYNQ_AVAILABLE:
            raise RuntimeError("pynq package not available -- this engine only runs on the board")
        bitstream = Path(bitstream)
        if not bitstream.exists() or not bitstream.with_suffix(".hwh").exists():
            raise FileNotFoundError(f"need {bitstream} and {bitstream.with_suffix('.hwh')} (make board-deploy)")
        self.timeout_s = timeout_s
        self.overlay = Overlay(str(bitstream))
        self.dma = getattr(self.overlay, dma_name)
        self.tx = allocate(shape=(MAX_TX_WORDS,), dtype=np.uint32)
        self.rx = allocate(shape=(MAX_RX_WORDS,), dtype=np.uint32)
        self.packets_sent = 0
        self.words_sent = 0

    # -- DMA plumbing ---------------------------------------------------------
    def _wait(self, channel, what: str) -> None:
        deadline = time.monotonic() + self.timeout_s
        while not channel.idle:
            if channel.error:
                raise RuntimeError(f"DMA {what} channel error (status register 0x{channel._mmio.read(channel._offset + 4):08x})")
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"DMA {what} did not complete within {self.timeout_s}s -- "
                    "no TLAST decision came back? (malformed packet, or pipeline stalled)"
                )

    def _exchange(self, words: list[int], n_decisions: int) -> list[Decision]:
        n = len(words)
        if n == 0:
            raise ValueError("a packet needs at least one word (use pack_sync for an empty window)")
        if n > MAX_TX_WORDS or n_decisions > MAX_RX_WORDS:
            raise ValueError(f"packet too large ({n} words, {n_decisions} decisions)")
        self.tx[:n] = np.asarray(words, dtype=np.uint32)
        self.rx[:n_decisions] = 0
        self.dma.recvchannel.transfer(self.rx[:n_decisions])
        self.dma.sendchannel.transfer(self.tx[:n])
        self._wait(self.dma.sendchannel, "MM2S (host -> FPGA)")
        self._wait(self.dma.recvchannel, "S2MM (FPGA -> host)")
        self.packets_sent += 1
        self.words_sent += n
        got = self.dma.recvchannel.transferred // 4
        return [unpack_decision(int(self.rx[i]), last=(i == got - 1)) for i in range(got)]

    # -- engine interface -----------------------------------------------------
    def step(self, words: list[int]) -> Decision:
        decisions = self._exchange(words, n_decisions=1)
        if len(decisions) != 1:
            raise RuntimeError(f"closed-loop packet produced {len(decisions)} decisions, expected 1")
        return decisions[0]

    def stream(self, words: list[int]) -> list[Decision]:
        # One decision per distinct window id in the packet (consecutive ids
        # from the event camera, so no catch-up windows), plus the reset
        # word's own decision if the packet is a bare reset.
        n_windows = len({(w & 0xFFFF) >> 8 for w in words}) if not _is_bare_reset(words) else 1
        return self._exchange(words, n_decisions=n_windows)

    def reset(self) -> Decision:
        return self.step([pack_reset(0)])

    def close(self) -> None:
        for buf in (getattr(self, "tx", None), getattr(self, "rx", None)):
            if buf is not None:
                buf.freebuffer()


def _is_bare_reset(words: list[int]) -> bool:
    return len(words) == 1 and (words[0] >> 30) == 0b01


if __name__ == "__main__":
    # Smoke test: reset, then one window with 30 left events -> TURN_R? No:
    # 30 events is bucket 1, below OBSTACLE_THRESH, so FORWARD with ev_left=1.
    from event_stream import pack_event, window_base
    eng = PynqEngine()
    try:
        print("reset ->", eng.reset())
        base = window_base(1)
        d = eng.step([pack_event(10, 64, i % 2, base + i) for i in range(30)])
        print("30 left events ->", d)
        assert (d.window_id, d.ev_left, d.count_left, d.state) == (1, 1, 30, 0), d
        base = window_base(2)
        d = eng.step([pack_event(10, 64, i % 2, base + i) for i in range(90)])
        print("90 left events ->", d)
        assert (d.ev_left, d.state) == (3, 2), d
        print("PYNQ smoke test OK")
    finally:
        eng.close()
