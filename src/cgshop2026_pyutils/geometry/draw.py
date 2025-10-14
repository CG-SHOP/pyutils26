import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.patches import Polygon
from ._bindings import Point

from .flippable_triangulation import FlippableTriangulation


def draw_edges(
    points: list[Point],
    edges: list[tuple[int, int]],
    ax: Axes | None = None,
    show_indices: bool = False,
):
    """
    Visualizes the given points and edges on the provided matplotlib Axes.

    Args:
        points: List of Point objects representing the vertices.
        edges: List of edges, where each edge is a tuple of vertex indices.
        ax: The matplotlib Axes to draw on. If None, a new figure and axes will be created.
        show_indices: If True, displays the indices of the points on the plot.
    """
    if ax is None:
        plt.figure()
        ax = plt.gca()

    # Draw edges
    for u, v in edges:
        ax.plot(
            [points[u].x(), points[v].x()],
            [points[u].y(), points[v].y()],
            color="black",
            linewidth=1,
        )

    # Draw points on top
    xs = [p.x() for p in points]
    ys = [p.y() for p in points]
    ax.scatter(xs, ys, color="black", s=20)

    # Optionally add indices to the points
    if show_indices:
        for idx, point in enumerate(points):
            ax.text(
                float(point.x()) + 0.05,
                float(point.y()) + 0.05,
                str(idx),
                color="blue",
                fontsize=12,
                ha="left",
                va="bottom",
            )

    ax.set_aspect("equal")
    ax.autoscale_view()


def draw_flips(
    triangulation: FlippableTriangulation,
    ax: Axes | None = None,
    show_indices: bool = False,
    title: str | None = None,
):
    """
    Visualizes the triangulation and highlights pending flips on the given matplotlib Axes.

    Args:
        triangulation: The FlippableTriangulation instance to visualize.
        ax: The matplotlib Axes to draw on. If None, a new figure and axes will be created.
        show_indices: If True, displays the indices of the points on the plot.
    """
    if ax is None:
        plt.figure()
        ax = plt.gca()
    points = triangulation._flip_map.points
    triangles = triangulation._flip_map.compute_triangles()
    flip_queue = triangulation._flip_queue

    # Draw triangles with light fill for better visibility
    for tri in triangles:
        polygon = Polygon(
            [[points[i].x(), points[i].y()] for i in tri],
            closed=True,
            facecolor="#e0e0e0",
            edgecolor="black",
            linewidth=1,
            alpha=0.4,
        )
        ax.add_patch(polygon)

    # Draw all edges in black
    drawn_edges = set()
    for tri in triangles:
        for i in range(3):
            u, v = tri[i], tri[(i + 1) % 3]
            edge = tuple(sorted((u, v)))
            if edge not in drawn_edges:
                ax.plot(
                    [points[u].x(), points[v].x()],
                    [points[u].y(), points[v].y()],
                    color="black",
                    linewidth=1,
                    zorder=1,
                )
                drawn_edges.add(edge)

    # Highlight pending flips in red and their flip partners in blue
    for edge in flip_queue:
        u, v = edge
        ax.plot(
            [points[u].x(), points[v].x()],
            [points[u].y(), points[v].y()],
            color="red",
            linewidth=1,
            zorder=3,
            label="Pending Flip" if edge == flip_queue[0] else "",
        )
        try:
            partner = triangulation.get_flip_partner(edge)
        except ValueError:
            partner = None
        if partner:
            pu, pv = partner
            ax.plot(
                [points[pu].x(), points[pv].x()],
                [points[pu].y(), points[pv].y()],
                color="blue",
                linewidth=1,
                linestyle="--",
                zorder=2,
                label="Flip Partner" if edge == flip_queue[0] else "",
            )

    # Draw points on top
    xs = [p.x() for p in points]
    ys = [p.y() for p in points]
    ax.scatter(xs, ys, color="black", s=20, zorder=4)

    # Optionally add indices to the points
    if show_indices:
        for idx, point in enumerate(points):
            ax.text(
                float(point.x()) + 0.05,
                float(point.y()) + 0.05,
                str(idx),
                color="blue",
                fontsize=12,
                ha="left",
                va="bottom",
                zorder=5,
            )

    ax.set_aspect("equal")
    if title:
        ax.set_title(title)
    ax.autoscale_view()
    # Add legend only once if flips exist
    if flip_queue:
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc="best")
