#!/usr/bin/env python3
"""
Create a matplotlib plot for a CGSHOP 2026 instance using only basic matplotlib functionality.

Layout:
- Top: point set plot, spanning *two columns* (i.e. twice as wide as triangulation panels).
- Below: triangulations, arranged in a grid with two panels per row.

Coordinate system is shown in all plots. Triangulations start counting from 0.

Usage:
- Use `create_instance_plot(instance, per_row=2)` to get a matplotlib figure for further processing.
"""

import math

import matplotlib.pyplot as plt
from .schemas import CGSHOP2026Instance


def _square_limits(xs: list[int], ys: list[int], pad_ratio: float = 0.05):
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    dx, dy = xmax - xmin, ymax - ymin
    dx = dx if dx > 0 else 1.0
    dy = dy if dy > 0 else 1.0
    span = max(dx, dy)
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    half = span / 2
    pad = pad_ratio * span
    return cx - half - pad, cx + half + pad, cy - half - pad, cy + half + pad


def _style_axes(ax, title: str | None = None):
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.tick_params(labelsize=6)
    ax.set_xlabel("x", fontsize=7)
    ax.set_ylabel("y", fontsize=7)
    if title:
        ax.set_title(title, fontsize=8, pad=4)


def create_instance_plot(inst: CGSHOP2026Instance, per_row: int = 2):
    xs, ys = inst.points_x, inst.points_y
    n_tris = len(inst.triangulations)
    cols = max(1, per_row)
    tri_rows = math.ceil(n_tris / cols) if n_tris > 0 else 0

    # GridSpec: top row spans 2 columns, then rows of 2
    fig = plt.figure(figsize=(12, 12 + 6 * tri_rows))
    import matplotlib.gridspec as gridspec

    gs = gridspec.GridSpec(
        1 + tri_rows, cols, figure=fig, height_ratios=[2] + [1] * tri_rows
    )

    # Top points panel
    ax_points = fig.add_subplot(gs[0, :])
    x0, x1, y0, y1 = _square_limits(xs, ys)
    ax_points.scatter(xs, ys, s=8)
    ax_points.set_xlim(x0, x1)
    ax_points.set_ylim(y0, y1)
    _style_axes(ax_points, title=f"{inst.instance_uid} — {len(xs)} points")

    # Triangulation panels
    for t_idx in range(n_tris):
        row = 1 + t_idx // cols
        col = t_idx % cols
        ax = fig.add_subplot(gs[row, col])
        ax.scatter(xs, ys, s=6, alpha=0.5)
        for i, j in inst.triangulations[t_idx]:
            ax.plot([xs[i], xs[j]], [ys[i], ys[j]], linewidth=0.8)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        _style_axes(ax, title=f"Triangulation #{t_idx}")

    fig.suptitle(f"CGSHOP2026 Instance: {inst.instance_uid}", fontsize=11, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.99]) # type: ignore
    return fig
