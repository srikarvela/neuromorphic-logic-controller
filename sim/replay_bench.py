"""Open-loop replay benchmark: one big packet, one decision per window.

A recorded episode (sim/cosim_driver.py --record) is streamed through the
pipeline as a single AXI-Stream packet -- TLAST only on the very last
word -- so windows close on timestamp boundaries alone and the hardware
emits one decision per window with no host round trip in between. The
decisions are checked against what the closed-loop run produced, and the
wall-clock time gives an events/s figure for the pipeline plus transport
(DMA on the board; pipe + simulator overhead under Icarus, where the number
only says the mechanism works).
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from engines import add_engine_args, engine_from_args
from event_stream import bytes_to_words

ROOT = Path(__file__).resolve().parent.parent


def load_expected(path: Path) -> list[dict]:
    with path.open() as f:
        return [{k: int(v) for k, v in row.items()} for row in csv.DictReader(f)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_engine_args(parser)
    parser.add_argument("--replay", type=Path, default=ROOT / "results" / "replay",
                        help="stem of <stem>.bin / <stem>.expected.csv from cosim_driver.py --record")
    parser.add_argument("--repeat", type=int, default=1, help="stream the packet this many times for timing")
    args = parser.parse_args()

    words = bytes_to_words(args.replay.with_suffix(".bin").read_bytes())
    expected = load_expected(args.replay.with_suffix(".expected.csv"))

    engine = engine_from_args(args)
    print(f"Engine: {engine.name}")
    try:
        mismatches = 0
        best = float("inf")
        for rep in range(args.repeat):
            engine.reset()
            t0 = time.perf_counter()
            decisions = engine.stream(words)
            dt = time.perf_counter() - t0
            best = min(best, dt)

            if len(decisions) != len(expected):
                raise RuntimeError(f"got {len(decisions)} decisions for {len(expected)} windows")
            for exp, d in zip(expected, decisions):
                if (d.ev_left, d.ev_right, d.state, d.brake_timer) != (
                    exp["ev_left"], exp["ev_right"], exp["state"], exp["brake_timer"]
                ):
                    mismatches += 1
                    print(f"[MISMATCH] window {exp['step']}: replay {d} vs closed-loop {exp}")
    finally:
        engine.close()

    n_events = sum(1 for w in words if w >> 31)
    print("----------------------------------------")
    print(f"{len(words)} words ({n_events} events, {len(expected)} windows) per pass, {args.repeat} pass(es)")
    print(f"best pass: {best * 1e3:.3f} ms  ->  {len(words) / best / 1e6:.3f} M words/s, "
          f"{len(expected) / best:.0f} decisions/s, {best / len(expected) * 1e6:.1f} us/window")
    print(f"{mismatches} mismatches against the closed-loop decisions")
    sys.exit(1 if mismatches else 0)


if __name__ == "__main__":
    main()
