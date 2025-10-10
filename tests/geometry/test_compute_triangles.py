"""
Unit tests for the compute_triangles function.

Tests verify that the function correctly computes triangles from
point sets and edge lists, handling various geometric configurations
including automatic convex hull edge addition.
"""

import pytest
from cgshop2026_pyutils.geometry import compute_triangles, Point


class TestComputeTriangles:
    """Test suite for the compute_triangles function."""

    def test_simple_triangle(self):
        """Test that a simple triangle returns one triangle."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []  # No edges - convex hull forms the triangle

        result = compute_triangles(points, edges)
        expected = [(0, 1, 2)]  # Single triangle with sorted indices

        assert result == expected, f"Expected {expected}, got {result}"

    def test_triangle_with_explicit_edges(self):
        """Test triangle with explicitly defined edges."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = [(0, 1), (1, 2), (2, 0)]

        result = compute_triangles(points, edges)
        expected = [(0, 1, 2)]

        assert result == expected, f"Expected {expected}, got {result}"

    def test_square_with_diagonal(self):
        """Test square triangulated with diagonal."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal from (0,0) to (1,1)

        result = compute_triangles(points, edges)
        # Should have two triangles
        expected = [(0, 1, 3), (0, 2, 3)]

        assert len(result) == 2, f"Expected 2 triangles, got {len(result)}"
        assert set(result) == set(expected), f"Expected {expected}, got {result}"

    def test_square_with_other_diagonal(self):
        """Test square triangulated with the other diagonal."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(1, 2)]  # Diagonal from (1,0) to (0,1)

        result = compute_triangles(points, edges)
        # Should have two triangles with the other diagonal
        expected = [(0, 1, 2), (1, 2, 3)]

        assert len(result) == 2, f"Expected 2 triangles, got {len(result)}"
        assert set(result) == set(expected), f"Expected {expected}, got {result}"

    def test_pentagon_fan_triangulation(self):
        """Test pentagon with fan triangulation from center."""
        points = [
            Point(0, 0),  # center
            Point(1, 0),  # right
            Point(0.5, 1),  # top-right
            Point(-0.5, 1),  # top-left
            Point(-1, 0),  # left
            Point(0, -1),  # bottom
        ]

        # Fan triangulation from center point
        edges = [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5)]

        result = compute_triangles(points, edges)

        # Should have 5 triangles in the fan
        assert len(result) == 5, f"Expected 5 triangles, got {len(result)}"

        # Each triangle should contain the center point (index 0)
        for triangle in result:
            assert 0 in triangle, f"Triangle {triangle} should contain center point"

    def test_empty_points_list(self):
        """Test behavior with empty points list."""
        points = []
        edges = []

        result = compute_triangles(points, edges)
        assert result == [], "Empty points should return empty triangles list"

    def test_single_point(self):
        """Test behavior with single point."""
        points = [Point(0, 0)]
        edges = []

        result = compute_triangles(points, edges)
        assert result == [], "Single point should return empty triangles list"

    def test_two_points(self):
        """Test behavior with two points."""
        points = [Point(0, 0), Point(1, 0)]
        edges = []

        result = compute_triangles(points, edges)
        assert result == [], "Two points should return empty triangles list"

    def test_collinear_points(self):
        """Test behavior with collinear points."""
        points = [Point(0, 0), Point(1, 0), Point(2, 0)]  # All on x-axis
        edges = []

        result = compute_triangles(points, edges)
        # Collinear points don't form proper triangles
        assert result == [], "Collinear points should return empty triangles list"

    def test_complex_triangulation(self):
        """Test a more complex triangulation."""
        # Create a hexagon-like point set
        points = [
            Point(0, 0),  # center
            Point(2, 0),  # right
            Point(1, 1.5),  # top-right
            Point(-1, 1.5),  # top-left
            Point(-2, 0),  # left
            Point(-1, -1.5),  # bottom-left
            Point(1, -1.5),  # bottom-right
        ]

        # Fan triangulation from center
        edges = [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6)]

        result = compute_triangles(points, edges)

        # Should have 6 triangles in the fan
        assert len(result) == 6, f"Expected 6 triangles, got {len(result)}"

        # Each triangle should contain the center point (index 0)
        for triangle in result:
            assert 0 in triangle, f"Triangle {triangle} should contain center point"
            assert len(set(triangle)) == 3, (
                f"Triangle {triangle} should have 3 unique vertices"
            )

    def test_edge_indices_out_of_bounds(self):
        """Test that invalid edge indices are handled."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = [(0, 1), (1, 5)]  # Index 5 is out of bounds

        with pytest.raises(RuntimeError, match="Edge indices are out of bounds"):
            compute_triangles(points, edges)

    def test_negative_edge_indices(self):
        """Test that negative edge indices are handled."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = [(0, 1), (-1, 2)]  # Negative index

        with pytest.raises(RuntimeError, match="Edge indices are out of bounds"):
            compute_triangles(points, edges)

    def test_triangles_have_sorted_indices(self):
        """Test that triangle indices are sorted within each triangle."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(3, 0)]  # Diagonal specified in reverse order

        result = compute_triangles(points, edges)

        # Check that all triangles have sorted indices
        for triangle in result:
            sorted_triangle = tuple(sorted(triangle))
            assert triangle == sorted_triangle, (
                f"Triangle {triangle} should have sorted indices"
            )

    def test_no_duplicate_triangles(self):
        """Test that no duplicate triangles are returned."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        # Add redundant edges
        edges = [(0, 1), (1, 2), (2, 0), (0, 1), (1, 2)]  # Some edges repeated

        result = compute_triangles(points, edges)

        # Should only have one unique triangle
        assert len(result) == 1, f"Expected 1 triangle, got {len(result)}"
        assert len(set(result)) == len(result), (
            "No duplicate triangles should be present"
        )

    def test_square_no_internal_edges(self):
        """Test square with no internal edges (should return empty)."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = []  # No internal edges, only convex hull

        result = compute_triangles(points, edges)

        # Square without diagonal cannot be triangulated
        assert result == [], (
            "Square without internal edges should return empty triangles list"
        )

    def test_result_is_sorted(self):
        """Test that the result list is sorted."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal

        result = compute_triangles(points, edges)
        sorted_result = sorted(result)

        assert result == sorted_result, (
            f"Result should be sorted: {result} vs {sorted_result}"
        )

    def test_large_triangulation(self):
        """Test a larger triangulation."""
        # Create a 3x3 grid of points
        points = []
        for i in range(3):
            for j in range(3):
                points.append(Point(i, j))

        # Create a fan triangulation from center point (index 4)
        center_idx = 4  # Point(1, 1)
        edges = []
        boundary_indices = [0, 1, 2, 3, 5, 6, 7, 8]  # All except center
        for idx in boundary_indices:
            edges.append((center_idx, idx))

        result = compute_triangles(points, edges)

        # Should have 8 triangles in the fan
        assert len(result) == 8, f"Expected 8 triangles, got {len(result)}"

        # Each triangle should contain the center point
        for triangle in result:
            assert center_idx in triangle, (
                f"Triangle {triangle} should contain center point"
            )

    def test_intersecting_edges_handled(self):
        """Test that intersecting edges are handled correctly."""
        # Create points that allow intersecting edges at an existing point
        points = [
            Point(0, 0),  # center intersection
            Point(-1, 1),  # top-left
            Point(1, 1),  # top-right
            Point(-1, -1),  # bottom-left
            Point(1, -1),  # bottom-right
        ]

        # Create intersecting edges that meet at the center point
        edges = [(1, 4), (2, 3)]  # These intersect at the origin point

        result = compute_triangles(points, edges)

        # Should produce triangles since intersection is at an existing point
        assert len(result) > 0, (
            "Should produce triangles when intersection is at existing point"
        )

        # All triangles should have 3 unique vertices
        for triangle in result:
            assert len(set(triangle)) == 3, (
                f"Triangle {triangle} should have 3 unique vertices"
            )

    def test_redundant_convex_hull_edges(self):
        """Test that redundant convex hull edges are handled correctly."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        # These edges are redundant since they're on the convex hull
        edges = [(0, 1), (1, 2), (2, 0)]

        result = compute_triangles(points, edges)
        expected = [(0, 1, 2)]

        assert result == expected, f"Expected {expected}, got {result}"

    def test_mixed_internal_and_boundary_edges(self):
        """Test triangulation with both internal and boundary edges."""
        points = [
            Point(0, 0),
            Point(2, 0),
            Point(1, 2),
            Point(1, 1),
        ]  # Triangle with internal point
        edges = [
            (0, 3),
            (1, 3),
            (2, 3),
        ]  # Connect internal point to all boundary points

        result = compute_triangles(points, edges)

        # Should have 3 triangles
        assert len(result) == 3, f"Expected 3 triangles, got {len(result)}"

        # Each triangle should contain the internal point (index 3)
        for triangle in result:
            assert 3 in triangle, f"Triangle {triangle} should contain internal point"
