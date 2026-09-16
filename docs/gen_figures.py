"""Regenerate the README figures from real hardware-in-the-loop runs.

- trajectory / event_windows / event_raster: a showcase course (obstacles
  on both sides) run here, closed loop, through the RTL under Icarus with
  every decision golden-checked. Needs build/tb_cosim_server.vvp.
- noise_sweep: reads results/noise_sweep.csv from sim/noise_sweep.py.

Outputs: docs/images/*.png.   Run: make figures
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from agent_sim import Environment, Obstacle  # noqa: E402
from cosim_driver import run_episode  # noqa: E402
from engines import IcarusEngine  # noqa: E402
from event_camera import EventCamera  # noqa: E402
from event_stream import RATE_THRESHOLDS, WINDOW_TICKS, X_CENTER, event_ts, event_x, is_event  # noqa: E402

OUT = ROOT / "docs" / "images"

# Reference palette (dataviz skill), light mode.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
OBSTACLE = "#c3c2b7"
S1, S2, S3, S4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
STATE_NAMES = ["FORWARD", "TURN_L", "TURN_R", "BRAKE"]
STATE_COLORS = {0: S1, 1: S2, 2: S3, 3: S4}

plt.rcParams.update({
    "font.family": ["system-ui", "-apple-system", "Helvetica Neue", "Arial", "DejaVu Sans"],
    "font.size": 10,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS,
    "axes.labelcolor": INK_2,
    "axes.titlecolor": INK,
    "axes.titlesize": 11,
    "axes.titleweight": "semibold",
    "axes.titlelocation": "left",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "xtick.labelcolor": INK_2,
    "ytick.labelcolor": INK_2,
    "legend.frameon": False,
    "legend.labelcolor": INK_2,
    "lines.linewidth": 2,
})


def read_csv(path: Path) -> list[dict]:
    with path.open() as f:
        return list(csv.DictReader(f))


def showcase_environment() -> Environment:
    # Right, then left, then right again: exercises both turn directions.
    return Environment(obstacles=[
        Obstacle(x=9.0, y=-0.6, radius=1.2),
        Obstacle(x=16.0, y=7.0, radius=1.3),
        Obstacle(x=28.0, y=3.0, radius=1.2),
    ])


def run_showcase() -> tuple[list[dict], list[int], Environment]:
    env = showcase_environment()
    words: list[int] = []
    with IcarusEngine() as engine:
        log, _ = run_episode(engine, env=env, max_steps=45, camera=EventCamera(seed=1), check=True, record=words)
    return log, words, env


def fig_trajectory(log: list[dict], env: Environment) -> None:
    fig, ax = plt.subplots(figsize=(10, 4.2), dpi=160)

    path_y = {round(float(r["x"])): float(r["y"]) for r in log}
    for i, obs in enumerate(env.obstacles):
        ax.add_patch(Circle((obs.x, obs.y), obs.radius, facecolor=OBSTACLE, edgecolor=SURFACE, linewidth=2, zorder=1))
        above = obs.y > path_y.get(round(obs.x), 0.0)  # label on the side away from the path
        ax.annotate(f"obstacle {i + 1}", (obs.x, obs.y + (obs.radius if above else -obs.radius)),
                    xytext=(0, 4 if above else -4), textcoords="offset points", color=INK_2, ha="center",
                    va="bottom" if above else "top", fontsize=9)

    xs = [0.0] + [float(r["x"]) for r in log]
    ys = [0.0] + [float(r["y"]) for r in log]
    states = [0] + [int(r["state"]) for r in log]
    ax.plot(xs, ys, color=AXIS, linewidth=1, zorder=2)
    seen = set()
    for s in sorted(set(states)):
        px = [x for x, st in zip(xs, states) if st == s]
        py = [y for y, st in zip(ys, states) if st == s]
        ax.scatter(px, py, s=36, color=STATE_COLORS[s], edgecolor=SURFACE, linewidth=1.2, zorder=3,
                   label=STATE_NAMES[s])
        seen.add(s)
    # direct labels on the first point of each turn run
    for i in range(1, len(states)):
        if states[i] != states[i - 1] and states[i] != 0:
            ax.annotate(STATE_NAMES[states[i]], (xs[i], ys[i]), xytext=(0, 10), textcoords="offset points",
                        color=INK, fontsize=9, ha="center")
    ax.annotate("start", (0, 0), xytext=(-8, 0), textcoords="offset points", color=INK_2, fontsize=9,
                ha="right", va="center")

    ax.set_aspect("equal")
    ax.set_xlim(-4, max(xs) + 2)
    ax.set_ylim(min(ys + [o.y - o.radius for o in env.obstacles]) - 2.5,
                max(ys + [o.y + o.radius for o in env.obstacles]) + 1.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Closed-loop episode: every decision came from the RTL")
    ax.legend(loc="upper left", title="FSM state", title_fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT / "trajectory.png")
    plt.close(fig)


def fig_event_windows(log: list[dict]) -> None:
    steps = [int(r["step"]) for r in log]
    left = [int(r["events_left"]) for r in log]
    right = [int(r["events_right"]) for r in log]
    states = [int(r["state"]) for r in log]

    fig, axes = plt.subplots(3, 1, figsize=(10, 5.6), dpi=160, sharex=True,
                             gridspec_kw={"height_ratios": [3, 3, 0.8]})
    for ax, counts, name in ((axes[0], left, "Left hemisphere"), (axes[1], right, "Right hemisphere")):
        ax.bar(steps, counts, width=0.8, color=S1, edgecolor=SURFACE, linewidth=1, zorder=2)
        for t, b in zip(RATE_THRESHOLDS, (1, 2, 3)):
            ax.axhline(t, color=MUTED, linewidth=1, linestyle=(0, (4, 3)), zorder=1)
            ax.annotate(f"≥{t} → bucket {b}", (1.0, t), xycoords=("axes fraction", "data"), xytext=(4, 0),
                        textcoords="offset points", color=INK_2, fontsize=8, va="center", annotation_clip=False)
        ax.set_ylim(0, 105)
        ax.set_ylabel("events / window")
        ax.set_title(f"{name}: events counted in hardware per 256-tick window")
        ax.grid(axis="x", visible=False)

    ax = axes[2]
    for s, st in zip(steps, states):
        ax.barh(0, 1, left=s - 0.5, height=0.8, color=STATE_COLORS[st], edgecolor=SURFACE, linewidth=1)
    runs = []
    start = 0
    for i in range(1, len(states) + 1):
        if i == len(states) or states[i] != states[start]:
            runs.append((start, i - 1, states[start]))
            start = i
    for a, b, st in runs:
        if st != 0:
            ax.annotate(STATE_NAMES[st], ((a + b) / 2, 0.45), xytext=(0, 4), textcoords="offset points",
                        ha="center", va="bottom", fontsize=8, color=INK, annotation_clip=False)
    ax.annotate("blue = FORWARD", (1.0, 0), xycoords=("axes fraction", "data"), xytext=(4, 0),
                textcoords="offset points", color=INK_2, fontsize=8, va="center", annotation_clip=False)
    ax.set_yticks([])
    ax.set_ylim(-0.5, 0.5)
    ax.grid(False)
    ax.spines["left"].set_visible(False)
    ax.set_xlabel("control step (one window each)")
    ax.set_title("FSM state after each window", fontsize=10, pad=16)
    axes[0].set_xlim(-0.8, steps[-1] + 0.8)
    fig.tight_layout(rect=(0, 0, 0.9, 1))
    fig.savefig(OUT / "event_windows.png")
    plt.close(fig)


def fig_event_raster(log: list[dict], words: list[int]) -> None:
    first, last = 0, 17
    fig, ax = plt.subplots(figsize=(10, 4), dpi=160)
    lx, ly, rx, ry = [], [], [], []
    for w in words:
        if not is_event(w):
            continue
        t = event_ts(w) / WINDOW_TICKS
        if not (first <= t < last + 1):
            continue
        (lx if event_x(w) < X_CENTER else rx).append(t)
        (ly if event_x(w) < X_CENTER else ry).append(event_x(w))
    ax.scatter(lx, ly, s=8, color=S1, linewidth=0, label="left hemisphere (x < 64)", zorder=3)
    ax.scatter(rx, ry, s=8, color=S2, linewidth=0, label="right hemisphere (x ≥ 64)", zorder=3)
    for k in range(first, last + 2):
        ax.axvline(k, color=GRID, linewidth=1, zorder=1)
    ax.axhline(X_CENTER, color=MUTED, linewidth=1, linestyle=(0, (4, 3)), zorder=2)
    ax.annotate("hemisphere split", (1.0, X_CENTER), xycoords=("axes fraction", "data"), xytext=(4, 0),
                textcoords="offset points", color=INK_2, fontsize=8, va="center", annotation_clip=False)
    for r in log[first:last + 1]:
        if int(r["state"]) != 0 and (int(r["step"]) == 0 or int(log[int(r["step"]) - 1]["state"]) == 0):
            ax.annotate(f"→ {r['action']}", (int(r["step"]) + 0.5, 127), xytext=(0, 4), textcoords="offset points",
                        ha="center", va="bottom", color=INK, fontsize=8)
    ax.set_xlim(first, last + 1)
    ax.set_ylim(0, 127)
    ax.invert_yaxis()
    ax.grid(False)
    ax.set_xlabel("time (windows; gridlines are window boundaries)")
    ax.set_ylabel("pixel column x")
    ax.set_title("What the FPGA receives: DVS address-events as an obstacle looms")
    ax.legend(loc="upper left", markerscale=2, ncols=2)
    fig.tight_layout(rect=(0, 0, 0.92, 1))
    fig.savefig(OUT / "event_raster.png")
    plt.close(fig)


def fig_noise() -> None:
    rows = read_csv(ROOT / "results" / "noise_sweep.csv")
    noise = [float(r["noise"]) for r in rows]
    series = [
        ("bucket_error_rate", "rate bucket wrong", S1),
        ("false_turn_rate", "turning with no obstacle", S2),
        ("false_brake_rate", "braking with no obstacle", S3),
    ]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4), dpi=160, gridspec_kw={"width_ratios": [1.6, 1]})
    for key, label, color in series:
        ys = [100 * float(r[key]) for r in rows]
        a1.plot(noise, ys, color=color, label=label, marker="o", markersize=5,
                markeredgecolor=SURFACE, markeredgewidth=1.2, zorder=3)
    a1.annotate("bucket wrong", (40, 99.5), xytext=(-6, 4), textcoords="offset points", color=INK, fontsize=8,
                ha="right", va="bottom")
    a1.annotate("false turns", (70, 95.4), xytext=(8, 0), textcoords="offset points", color=INK, fontsize=8,
                va="center")
    a1.annotate("false brakes", (100, 82.8), xytext=(0, 8), textcoords="offset points", color=INK, fontsize=8,
                ha="right", va="bottom")
    a1.axvspan(-2, 10, color=GRID, alpha=0.5, zorder=0, linewidth=0)
    a1.set_xlim(-2, 102)
    a1.set_ylim(0, 105)
    a1.set_xlabel("background noise (mean events per hemisphere per window)")
    a1.set_ylabel("% of windows")
    a1.set_title("Sensor noise vs. decisions")
    a1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncols=3, fontsize=8)

    col = [100 * float(r["collision_rate"]) for r in rows]
    a2.bar(noise, col, width=7, color=S1, edgecolor=SURFACE, linewidth=1, zorder=2)
    for x, y in zip(noise, col):
        if y > 0:
            a2.annotate(f"{y:.0f}%", (x, y), xytext=(0, 3), textcoords="offset points", ha="center",
                        color=INK, fontsize=8)
    a2.set_ylim(0, 40)
    a2.set_xlim(-6, 106)
    a2.set_xlabel("background noise")
    a2.set_ylabel("% of episodes")
    a2.set_title("Collisions (100 courses each)")
    a2.grid(axis="x", visible=False)
    fig.tight_layout()
    fig.savefig(OUT / "noise_sweep.png", bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    log, words, env = run_showcase()
    fig_trajectory(log, env)
    fig_event_windows(log)
    fig_event_raster(log, words)
    fig_noise()
    for p in sorted(OUT.glob("*.png")):
        print(f"wrote {p.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
