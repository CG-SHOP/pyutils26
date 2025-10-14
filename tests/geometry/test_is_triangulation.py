"""
Unit tests for the is_triangulation function.

Tests verify that the function correctly identifies valid triangulations
and rejects invalid configurations including non-triangular faces,
intersecting edges, and malformed input.
"""

import pytest
from cgshop2026_pyutils.geometry import is_triangulation, Point


class TestIsTriangulation:
    """Test suite for the is_triangulation function."""

    def test_valid_triangle(self):
        """Test that a simple triangle is correctly identified as a triangulation."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        # No edges needed - convex hull forms the triangle automatically
        edges = []
        assert is_triangulation(points, edges), (
            "Simple triangle should be a valid triangulation"
        )

    def test_valid_triangle_with_explicit_edges(self):
        """Test triangle with explicitly defined edges."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = [(0, 1), (1, 2), (2, 0)]
        assert is_triangulation(points, edges), (
            "Triangle with explicit edges should be valid"
        )

    def test_valid_square_triangulation(self):
        """Test that a properly triangulated square is valid."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        # Diagonal splits square into two triangles
        edges = [(0, 3)]  # Diagonal from (0,0) to (1,1)
        assert is_triangulation(points, edges), (
            "Square with diagonal should be a valid triangulation"
        )

    def test_valid_square_triangulation_alternative_diagonal(self):
        """Test square triangulated with the other diagonal."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        # Alternative diagonal splits square into two triangles
        edges = [(1, 2)]  # Diagonal from (1,0) to (0,1)
        assert is_triangulation(points, edges), (
            "Square with alternative diagonal should be valid"
        )

    def test_valid_square_triangulation_explicit_edges(self):
        """Test square with all edges explicitly defined."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 1), (1, 3), (3, 2), (2, 0), (0, 3)]  # All edges including diagonal
        assert is_triangulation(points, edges), (
            "Square with all explicit edges should be valid"
        )

    def test_invalid_square_no_diagonal(self):
        """Test that a square without diagonal is not a triangulation."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = []  # No internal edges, only convex hull
        assert not is_triangulation(points, edges), (
            "Square without diagonal should not be a triangulation"
        )

    def test_invalid_square_explicit_boundary_only(self):
        """Test square with only boundary edges (no triangulation)."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 1), (1, 3), (3, 2), (2, 0)]  # Only boundary, no diagonal
        assert not is_triangulation(points, edges), (
            "Square with only boundary should not be a triangulation"
        )

    def test_valid_pentagon_triangulation(self):
        """Test a properly triangulated pentagon."""
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
        assert is_triangulation(points, edges), (
            "Pentagon with fan triangulation should be valid"
        )

    def test_intersecting_edges_invalid(self):
        """Test that intersecting edges create an invalid triangulation."""
        points = [Point(0, 0), Point(2, 0), Point(1, 1), Point(1, -1)]
        # These edges intersect at (1, 0)
        edges = [(0, 2), (1, 3)]
        assert not is_triangulation(points, edges), (
            "Intersecting edges should create invalid triangulation"
        )

    def test_t_intersection_invalid(self):
        """Test that t-intersections are correctly identified as invalid."""
        # Create a configuration where one edge ends exactly at a point on another edge
        # This creates a t-intersection which is prohibited in valid triangulations
        points = [
            Point(0, 0),  # bottom-left
            Point(2, 0),  # bottom-right
            Point(1, 0),  # middle-bottom (on the edge between points 0 and 1)
            Point(1, 1),  # top-middle
        ]

        # Create edges that form a t-intersection:
        # - Edge (0, 1) goes from (0,0) to (2,0) - horizontal base
        # - Edge (2, 3) goes from (1,0) to (1,1) - vertical line ending exactly on the base
        # This creates a T-shaped intersection at point (1,0)
        edges = [(0, 1), (2, 3)]

        # This should be invalid because edge (2,3) creates a t-intersection
        # with edge (0,1) at point (1,0)
        assert not is_triangulation(points, edges), (
            "T-intersections should be invalid in triangulations"
        )

    def test_collinear_points(self):
        """Test triangulation with collinear points."""
        points = [Point(0, 0), Point(1, 0), Point(2, 0)]  # All on x-axis
        edges = []
        # Collinear points cannot form a proper triangulation (degenerate)
        # The function should handle this gracefully
        result = is_triangulation(points, edges)
        # This is expected to be False as collinear points don't form a 2D triangulation
        assert not result, "Collinear points should not form a valid triangulation"

    def test_duplicate_points_invalid(self):
        """Test that duplicate points are handled correctly."""
        points = [
            Point(0, 0),
            Point(1, 0),
            Point(0, 1),
            Point(0, 0),
        ]  # Last point is duplicate
        edges = [(0, 1), (1, 2), (2, 0)]
        # This should either handle duplicates gracefully or raise an error
        # Based on the C++ code, it appears to check for this
        try:
            result = is_triangulation(points, edges)
            # If it doesn't raise an error, it should return False
            assert not result, "Duplicate points should not create valid triangulation"
        except Exception:
            # If it raises an exception, that's also acceptable behavior
            pass

    def test_duplicate_points_wo_edges(self):
        points = [
            Point(0, 0),
            Point(1, 0),
            Point(0, 1),
            Point(0, 0),
        ]  # Last point is duplicate
        edges = []
        assert not is_triangulation(points, edges), (
            "Duplicate points should not create valid triangulation"
        )

    def test_edge_indices_out_of_bounds(self):
        """Test that invalid edge indices are handled."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = [(0, 1), (1, 5)]  # Index 5 is out of bounds

        with pytest.raises(RuntimeError, match="Edge indices are out of bounds"):
            is_triangulation(points, edges)

    def test_negative_edge_indices(self):
        """Test that negative edge indices are handled."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = [(0, 1), (-1, 2)]  # Negative index

        with pytest.raises(RuntimeError, match="Edge indices are out of bounds"):
            is_triangulation(points, edges)

    def test_empty_points_list(self):
        """Test behavior with empty points list."""
        points = []
        edges = []

        # Empty point set is a valid degenerate triangulation
        result = is_triangulation(points, edges)
        assert result, "Empty point set should be a valid degenerate triangulation"

    def test_single_point(self):
        """Test behavior with single point."""
        points = [Point(0, 0)]
        edges = []

        # Single point cannot form a triangulation
        result = is_triangulation(points, edges)
        assert not result, "Single point should not form a valid triangulation"

    def test_two_points(self):
        """Test behavior with two points."""
        points = [Point(0, 0), Point(1, 0)]
        edges = []

        # Two points form a degenerate but valid triangulation
        result = is_triangulation(points, edges)
        assert result, "Two points should form a valid (degenerate) triangulation"

    def test_complex_valid_triangulation(self):
        """Test a more complex valid triangulation."""
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

        assert is_triangulation(points, edges), (
            "Complex fan triangulation should be valid"
        )

    def test_valid_delaunay_like_triangulation(self):
        """Test a triangulation that resembles Delaunay triangulation properties."""
        points = [Point(0, 0), Point(3, 0), Point(1.5, 2), Point(1.5, -1)]

        # Connect all points to form triangles
        edges = [
            (0, 2),
            (1, 3),
            (2, 3),
        ]  # Internal edges, convex hull added automatically

        assert is_triangulation(points, edges), (
            "Delaunay-like triangulation should be valid"
        )

    def test_non_triangular_face_invalid(self):
        """Test that non-triangular faces make triangulation invalid."""
        # Create a pentagon
        points = [
            Point(1, 0),  # right
            Point(0.5, 1),  # top-right
            Point(-0.5, 1),  # top-left
            Point(-1, 0),  # left
            Point(0, -1),  # bottom
        ]

        # No internal edges - this creates a single pentagonal face
        edges = []

        assert not is_triangulation(points, edges), (
            "Pentagon without triangulation should be invalid"
        )

    def test_self_intersecting_boundary_invalid(self):
        """Test that self-intersecting configurations are invalid triangulations."""
        # Create a bow-tie or figure-8 like configuration
        points = [
            Point(0, 0),  # center intersection
            Point(-1, 1),  # top-left
            Point(1, 1),  # top-right
            Point(-1, -1),  # bottom-left
            Point(1, -1),  # bottom-right
        ]

        # Create intersecting edges that meet at the center point
        edges = [(1, 4), (2, 3)]  # These intersect at the origin point
        # This forms an invalid triangulation because the edges cross at (0,0),
        # violating the rule that edges must not intersect except at their endpoints.
        assert not is_triangulation(points, edges), (
            "The edges cross on (0,0), making it invalid"
        )

    def test_truly_intersecting_edges_invalid(self):
        """Test that edges creating new intersection points are invalid."""
        # Create points that force edge intersection at a point not in the set
        points = [
            Point(0, 0),  # bottom-left
            Point(2, 0),  # bottom-right
            Point(0, 2),  # top-left
            Point(2, 2),  # top-right
        ]

        # These edges will intersect at (1, 1) which is not in our point set
        edges = [(0, 3), (1, 2)]  # Diagonal edges that cross

        # This should be invalid because it creates a new intersection point
        assert not is_triangulation(points, edges), (
            "Edges creating new intersections should be invalid"
        )

    def test_large_triangulation_valid(self):
        """Test a larger valid triangulation."""
        # Create a grid of points
        points = []
        for i in range(3):
            for j in range(3):
                points.append(Point(i, j))

        # Create a fan triangulation from center point
        center_idx = 4  # Point(1, 1) is at index 4
        edges = []

        # Connect center to all boundary points
        boundary_indices = [0, 1, 2, 3, 5, 6, 7, 8]  # All except center
        for idx in boundary_indices:
            edges.append((center_idx, idx))

        assert is_triangulation(points, edges), (
            "Large fan triangulation should be valid"
        )

    def test_empty_edges_triangle_valid(self):
        """Test that empty edges list works for triangular point sets."""
        points = [Point(0, 0), Point(3, 0), Point(1, 2)]
        edges = []  # No edges - convex hull will be added automatically

        assert is_triangulation(points, edges), (
            "Triangle with empty edges should be valid"
        )

    def test_redundant_edges_invalid(self):
        """Test that redundant edges (already on convex hull) are not valid."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        # These edges are redundant since they're on the convex hull
        edges = [(0, 1), (1, 2), (2, 0), (0, 1)]  # Duplicate edge

        assert not is_triangulation(points, edges), (
            "Redundant convex hull edges should not be valid"
        )

    def test_verbose_flag_functionality(self):
        """Test that verbose flag can be set and function still works correctly."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []

        # Test with verbose=False (default)
        result_quiet = is_triangulation(points, edges)
        assert result_quiet, "Triangle should be valid with verbose=False"

        # Test with verbose=True
        result_verbose = is_triangulation(points, edges, verbose=True)
        assert result_verbose, "Triangle should be valid with verbose=True"

        # Results should be the same regardless of verbose setting
        assert result_quiet == result_verbose, (
            "Results should be consistent regardless of verbose setting"
        )

    def test_verbose_parameter_types(self):
        """Test that verbose parameter accepts different ways of being specified."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []

        # Test with keyword argument
        result1 = is_triangulation(points, edges, verbose=False)

        # Test with positional argument
        result2 = is_triangulation(points, edges, True)

        # Test default (omitted)
        result3 = is_triangulation(points, edges)

        # All should work and give consistent results
        assert result1, "Should work with verbose=False keyword"
        assert result2, "Should work with verbose=True positional"
        assert result3, "Should work with default verbose"

    def test_nasty_overlap(self):
        points = [
            Point(0, 0),
            Point(5, 0),
            Point(10, 0),
            Point(10, 1),
            Point(0, 1),
        ]
        edges = [(0, 2), (0, 1), (0, 4), (1, 4), (1, 3)]
        assert not is_triangulation(points, edges), (
            "The edge (0,2) should be illegal as it overlaps with the point 1"
        )

    def test_nasty_overlap2(self):
        points = [
            Point(0, 0),
            Point(5, 0),
            Point(10, 0),
            Point(10, 1),
            Point(0, 1),
        ]
        edges = [(0, 1), (0, 2), (0, 4), (1, 4), (1, 3)]
        assert not is_triangulation(points, edges), (
            "The edge (0,2) should be illegal as it overlaps with the point 1"
        )

    def test_nasty_overlap3(self):
        points = [
            Point(0, 0),  # 0
            Point(1, 0),  # 1
            Point(2, 0),  # 2
            Point(2, 2),  # 3
            Point(-2, 2),  # 4
            Point(-2, -2),  # 5
            Point(2, -2),  # 6
        ]
        edges = [
            (0, 2),
            (0, 1),
            (0, 4),
            (0, 5),
            (0, 6),
            (1, 6),
            (1, 3),
            (0, 3),
        ]
        assert not is_triangulation(points, edges), (
            "Nasty overlapping edges (0,2) and (0,1) should be invalid"
        )
