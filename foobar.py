from cgshop2026_pyutils.geometry import is_triangulation, Point


def test_is_triangulation():
    # Remember: The convex hull edges are added automatically, but you can also specify them manually.
    # Intersecting edges are not allowed.

    points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
    edges = [(0, 3), (1, 2)]  # Square, not triangulated
    assert not is_triangulation(points, edges), "Square should not be a triangulation"

    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]  # Another square, not triangulated
    assert not is_triangulation(points, edges), (
        "Another square should not be a triangulation"
    )

    edges = [(0, 1), (1, 3), (3, 2), (2, 0), (0, 2)]  # Triangulated square
    assert is_triangulation(points, edges), (
        "Triangulated square should be a triangulation"
    )


if __name__ == "__main__":
    test_is_triangulation()
    print("All tests passed.")
