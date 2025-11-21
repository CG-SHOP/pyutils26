"""
CGAL geometry bindings for CG:SHOP 2026
"""

from typing import overload, Sequence
from typing_extensions import Self, override
class FieldNumber:
    """A container for exact numbers in CGAL."""

    @overload
    def __init__(self, value: int) -> None: ...
    @overload
    def __init__(self, value: float) -> None: ...
    @overload
    def __init__(self, value: str) -> None: ...
    def __truediv__(self, other: Self) -> Self: ...
    def __add__(self, other: Self) -> Self: ...
    def __sub__(self, other: Self) -> Self: ...
    def __mul__(self, other: Self) -> Self: ...
    @override
    def __eq__(self, other: object) -> bool: ...
    def __lt__(self, other: Self) -> bool: ...
    def __gt__(self, other: Self) -> bool: ...
    def __le__(self, other: Self) -> bool: ...
    def __ge__(self, other: Self) -> bool: ...
    def __float__(self) -> float: ...
    @override
    def __str__(self) -> str: ...
    def exact(self) -> str: ...

class Point:
    """A 2-dimensional point."""

    @overload
    def __init__(self, x: int, y: int) -> None: ...
    @overload
    def __init__(self, x: float, y: float) -> None: ...
    @overload
    def __init__(self, x: FieldNumber, y: FieldNumber) -> None: ...
    def __add__(self, other: Point) -> Point: ...
    def __sub__(self, other: Point) -> Point: ...
    @override
    def __eq__(self, other: object) -> bool: ...
    @override
    def __ne__(self, other: object) -> bool: ...
    def x(self) -> FieldNumber: ...
    def y(self) -> FieldNumber: ...
    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> FieldNumber: ...
    @override
    def __str__(self) -> str: ...

class Segment:
    """A 2-dimensional segment."""

    def __init__(self, source: Point, target: Point) -> None: ...
    def source(self) -> Point: ...
    def target(self) -> Point: ...
    @override
    def __str__(self) -> str: ...

def is_triangulation(
    points: Sequence[Point], edges: Sequence[tuple[int, int]], verbose: bool = False
) -> bool:
    """
    Check if a set of edges forms a triangulation of the given points.

    Uses the CGAL arrangement data structure to insert the edges and verify
    the triangulation properties.

    Args:
        points: A sequence of Point objects representing the vertices.
        edges: A sequence of (int, int) tuples representing edges as point indices.
        verbose: If True, print additional information during validation.

    Returns:
        True if the edges form a valid triangulation, False otherwise.
    """
    ...

def compute_triangles(
    points: Sequence[Point], edges: Sequence[tuple[int, int]]
) -> list[tuple[int, int, int]]:
    """
    Compute all triangles formed by the given set of points and edges.

    Returns a list of triangles, where each triangle is represented by a tuple
    of three point indices. Edges that appear only once will be on the convex
    hull. Otherwise, all edges should appear exactly twice. The indices will be
    sorted in each triangle, and the list of triangles will also be sorted.

    Args:
        points: A sequence of Point objects representing the vertices.
        edges: A sequence of (int, int) tuples representing edges as point indices.

    Returns:
        A sorted list of triangles, each represented as a tuple of three sorted
        point indices (int, int, int).
    """
    ...

def do_cross(seg1: Segment, seg2: Segment) -> bool:
    """
    Check if two segments cross each other.

    Two segments cross if they intersect in a point that is not an endpoint.
    No endpoint is allowed to lie on the other segment.

    Args:
        seg1: The first segment.
        seg2: The second segment.

    Returns:
        True if the segments cross, False otherwise.
    """
    ...
