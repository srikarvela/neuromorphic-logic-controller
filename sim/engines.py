"""Hardware engines: where the FSM decisions actually come from.

Every engine speaks the same contract -- a packet of event words in, a
decision word out -- so sim/cosim_driver.py and sim/stress_test.py don't
know or care whether the "chip" is:

- IcarusEngine  the RTL (rtl/nlc_axis_top.sv) running in one persistent
                Icarus Verilog process, packets exchanged over stdin/stdout
- PynqEngine    the same RTL synthesized onto the PYNQ-Z2's XC7Z020,
                packets exchanged over AXI DMA (board/pynq/nlc_pynq.py)
- GoldenPipeline the pure-Python reference model (sim/golden_model.py),
                only ever used to cross-check the two above

    engine.reset()          -> Decision    in-band reset, FSM back to FORWARD
    engine.step(words)      -> Decision    one closed-loop control step
    engine.stream(words)    -> [Decision]  one packet, many windows (replay)
    engine.close()
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Protocol

from event_stream import Decision, pack_reset, unpack_decision

ROOT = Path(__file__).resolve().parent.parent
VVP_BIN = ROOT / "build" / "tb_cosim_server.vvp"

ENGINE_NAMES = ("icarus", "pynq", "golden")


class Engine(Protocol):
    name: str

    def reset(self) -> Decision: ...
    def step(self, words: list[int]) -> Decision: ...
    def stream(self, words: list[int]) -> list[Decision]: ...
    def close(self) -> None: ...


class IcarusEngine:
    """Drives tb/tb_cosim_server.sv: one vvp process for the whole session."""

    name = "RTL under Icarus Verilog"

    def __init__(self, vvp_bin: Path = VVP_BIN, vcd: Path | None = None):
        if not vvp_bin.exists():
            raise FileNotFoundError(f"{vvp_bin} not found -- run scripts/build_cosim.sh first")
        cmd = ["vvp", "-n", str(vvp_bin)]
        if vcd is not None:
            cmd.append(f"+vcd={vcd}")
        self.proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
        self._expect("READY")
        self.beats_sent = 0

    def _expect(self, token: str) -> None:
        while True:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError("cosim server exited before READY")
            if line.strip() == token:
                return

    def _read_decisions(self) -> list[Decision]:
        out: list[Decision] = []
        while True:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError("cosim server exited mid-packet")
            line = line.strip()
            if line.startswith("RESULT "):
                fields = dict(kv.split("=") for kv in line[7:].split())
                d = unpack_decision(int(fields["word"], 16), last=fields["last"] == "1")
                out.append(d)
                if d.last:
                    return out
            elif line.startswith("ERROR"):
                raise RuntimeError(f"cosim server: {line}")
            # anything else (VCD info etc.) is ignored

    def stream(self, words: list[int]) -> list[Decision]:
        if not words:
            raise ValueError("a packet needs at least one word (use pack_sync for an empty window)")
        lines = [f"{w & 0xFFFFFFFF:08x} {1 if i == len(words) - 1 else 0}\n" for i, w in enumerate(words)]
        self.proc.stdin.write("".join(lines))
        self.proc.stdin.flush()
        self.beats_sent += len(words)
        return self._read_decisions()

    def step(self, words: list[int]) -> Decision:
        decisions = self.stream(words)
        if len(decisions) != 1:
            raise RuntimeError(f"closed-loop packet produced {len(decisions)} decisions, expected 1")
        return decisions[0]

    def reset(self) -> Decision:
        return self.step([pack_reset(0)])

    def close(self) -> None:
        if self.proc.poll() is None:
            try:
                self.proc.stdin.write("Q\n")
                self.proc.stdin.flush()
            except (BrokenPipeError, ValueError):
                pass
            self.proc.wait(timeout=10)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def make_engine(name: str, **kwargs) -> Engine:
    if name == "icarus":
        return IcarusEngine(**kwargs)
    if name == "golden":
        from golden_model import GoldenPipeline
        return GoldenPipeline()
    if name == "pynq":
        sys.path.insert(0, str(ROOT / "board" / "pynq"))
        from nlc_pynq import PynqEngine
        return PynqEngine(**kwargs)
    raise ValueError(f"unknown engine {name!r}; choose from {ENGINE_NAMES}")


def add_engine_args(parser) -> None:
    parser.add_argument("--engine", choices=ENGINE_NAMES, default="icarus",
                        help="where decisions come from (default: RTL under Icarus)")
    parser.add_argument("--bitstream", type=Path, default=None,
                        help="(pynq) path to nlc.bit; default board/pynq/nlc.bit next to its .hwh")
    parser.add_argument("--no-check", action="store_true",
                        help="skip cross-checking every hardware decision against the golden model")


def engine_from_args(args) -> Engine:
    kwargs = {}
    if args.engine == "pynq" and args.bitstream is not None:
        kwargs["bitstream"] = args.bitstream
    return make_engine(args.engine, **kwargs)
