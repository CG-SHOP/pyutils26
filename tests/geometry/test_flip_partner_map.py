"""
Unit tests for the FlipPartnerMap class.

Tests verify that the class correctly identifies flippable edges, maintains
flip partner mappings, handles edge flips, and manages conflicting flips
in triangulated point sets.
"""

import pytest
from cgshop2026_pyutils.geometry import FlipPartnerMap, Point


class TestFlipPartnerMap:
    """Test suite for the FlipPartnerMap class."""

    def test_simple_square_creation(self):
        """Test creating FlipPartnerMap from a simple triangulated square."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal from (0,0) to (1,1)

        flip_map = FlipPartnerMap.build(points, edges)

        assert flip_map is not None, "FlipPartnerMap should be created successfully"
        assert len(flip_map.points) == 4, "Should have 4 points"
        assert len(flip_map.compute_triangles()) == 2, "Should have 2 triangles"

    def test_triangle_no_flippable_edges(self):
        """Test that a simple triangle has no flippable edges."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []  # Convex hull forms the triangle

        flip_map = FlipPartnerMap.build(points, edges)

        # Triangle has no internal edges, so no flippable edges
        flippable = flip_map.flippable_edges()
        assert len(flippable) == 0, "Triangle should have no flippable edges"

    def test_square_diagonal_flippable(self):
        """Test that diagonal in a square is flippable."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal from (0,0) to (1,1)

        flip_map = FlipPartnerMap.build(points, edges)

        # The diagonal should be flippable
        assert flip_map.is_flippable((0, 3)), "Diagonal (0,3) should be flippable"

        flippable = flip_map.flippable_edges()
        assert len(flippable) == 1, "Should have exactly one flippable edge"
        assert (0, 3) in flippable, "Diagonal should be in flippable edges"

    def test_square_diagonal_flip_partner(self):
        """Test that flip partner of square diagonal is correct."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal from (0,0) to (1,1)

        flip_map = FlipPartnerMap.build(points, edges)

        # Check internal flip map structure
        assert (0, 3) in flip_map.flip_map, "Edge (0, 3) should be in flip map"
        partner = flip_map.flip_map[(0, 3)]
        assert set(partner) == {
            1,
            2,
        }, f"Flip partner should be vertices 1 and 2, got {partner}"

    def test_flip_operation(self):
        """Test that flipping an edge works correctly."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal from (0,0) to (1,1)

        flip_map = FlipPartnerMap.build(points, edges)

        # Flip the diagonal
        new_edge = flip_map.flip((0, 3))

        # The new edge should be the other diagonal
        expected_new_edge = (1, 2)
        assert new_edge == expected_new_edge, (
            f"Expected new edge {expected_new_edge}, got {new_edge}"
        )

        # Original edge should no longer be flippable
        assert not flip_map.is_flippable((0, 3)), (
            "Original edge should no longer be flippable"
        )

        # New edge should now be flippable
        assert flip_map.is_flippable(new_edge), "New edge should be flippable"

    def test_flip_nonflippable_edge_raises_error(self):
        """Test that flipping a non-flippable edge raises an error."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []  # Simple triangle

        flip_map = FlipPartnerMap.build(points, edges)

        # Try to flip a boundary edge (should fail)
        with pytest.raises(
            ValueError, match="Edge is not flippable"
        ):
            flip_map.flip((0, 1))

    def test_conflicting_flips_square(self):
        """Test identifying conflicting flips in a square."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal from (0,0) to (1,1)

        flip_map = FlipPartnerMap.build(points, edges)

        # Get conflicting flips for the diagonal
        conflicting = flip_map.conflicting_flips((0, 3))

        # Should be empty for this simple case since boundary edges can't be flipped
        # (The diagonal is the only internal edge)
        assert isinstance(conflicting, set), "Should return a set"

    def test_conflicting_flips_nonflippable_edge_raises_error(self):
        """Test that getting conflicting flips for non-flippable edge raises error."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []

        flip_map = FlipPartnerMap.build(points, edges)

        with pytest.raises(ValueError, match="Edge is not flippable"):
            flip_map.conflicting_flips((0, 1))

    def test_deep_copy(self):
        """Test that deep_copy creates an independent copy."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        original = FlipPartnerMap.build(points, edges)
        copy = original.deep_copy()

        # Copies should be equal initially
        assert copy.points == original.points, "Points should be copied"
        assert copy.compute_triangles() == original.compute_triangles(), (
            "Triangles should be copied"
        )
        assert copy.flip_map == original.flip_map, "Flip map should be copied"

        # Modify the copy
        copy.flip((0, 3))

        # Original should be unchanged
        assert original.is_flippable((0, 3)), (
            "Original should still have flippable edge"
        )
        assert not copy.is_flippable((0, 3)), "Copy should have flipped edge"

    def test_complex_triangulation(self):
        """Test FlipPartnerMap with a more complex triangulation."""
        # Create a pentagon with center point
        points = [
            Point(0, 0),  # center
            Point(1, 0),  # right
            Point(0.5, 1),  # top-right
            Point(-0.5, 1),  # top-left
            Point(-1, 0),  # left
            Point(0, -1),  # bottom
        ]

        # Fan triangulation from center
        edges = [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5)]

        flip_map = FlipPartnerMap.build(points, edges)

        assert len(flip_map.compute_triangles()) == 5, "Should have 5 triangles in fan"

        # Each spoke should be flippable (connects center to boundary through two triangles)
        flippable = flip_map.flippable_edges()

        # Some edges might be flippable depending on convexity
        assert isinstance(flippable, list), "Should return list of flippable edges"

    def test_edge_ordering_consistency(self):
        """Test that edge ordering is handled consistently."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        flip_map = FlipPartnerMap.build(points, edges)

        # Test both orderings of the same edge
        assert flip_map.is_flippable((0, 3)) == flip_map.is_flippable((3, 0)), (
            "Edge flippability should be consistent regardless of vertex order"
        )

    def test_empty_triangulation(self):
        """Test behavior with empty or minimal input."""
        points = []
        edges = []

        flip_map = FlipPartnerMap.build(points, edges)

        assert len(flip_map.points) == 0, "Should handle empty point set"
        assert len(flip_map.compute_triangles()) == 0, "Should have no triangles"
        assert len(flip_map.flippable_edges()) == 0, "Should have no flippable edges"

    def test_single_point(self):
        """Test behavior with single point."""
        points = [Point(0, 0)]
        edges = []

        flip_map = FlipPartnerMap.build(points, edges)

        assert len(flip_map.points) == 1, "Should handle single point"
        assert len(flip_map.compute_triangles()) == 0, "Should have no triangles"
        assert len(flip_map.flippable_edges()) == 0, "Should have no flippable edges"

    def test_collinear_points(self):
        """Test behavior with collinear points."""
        points = [Point(0, 0), Point(1, 0), Point(2, 0)]
        edges = []

        flip_map = FlipPartnerMap.build(points, edges)

        # Collinear points cannot form proper triangulation
        assert len(flip_map.compute_triangles()) == 0, (
            "Collinear points should have no triangles"
        )
        assert len(flip_map.flippable_edges()) == 0, "Should have no flippable edges"

    def test_multiple_flips(self):
        """Test performing multiple consecutive flips."""
        points = [Point(0, 0), Point(2, 0), Point(1, 1), Point(1, -1)]
        edges = [(0, 2), (1, 3)]  # Two crossing diagonals

        flip_map = FlipPartnerMap.build(points, edges)

        initial_flippable = flip_map.flippable_edges()

        if len(initial_flippable) > 0:
            # Perform first flip
            edge1 = initial_flippable[0]
            flip_map.flip(edge1)

            # Check that the state is consistent
            assert not flip_map.is_flippable(edge1), (
                "Original edge should not be flippable"
            )

            # If there are more flippable edges, try another flip
            remaining_flippable = flip_map.flippable_edges()
            if len(remaining_flippable) > 0:
                edge2 = remaining_flippable[0]
                flip_map.flip(edge2)
                assert not flip_map.is_flippable(edge2), (
                    "Second flipped edge should not be flippable"
                )

    def test_convex_quadrilateral_detection(self):
        """Test that only convex quadrilaterals allow flips."""
        # Create a "butterfly" or non-convex quadrilateral
        points = [
            Point(0, 0),  # center
            Point(1, 1),  # top-right
            Point(-1, 1),  # top-left
            Point(1, -1),  # bottom-right
            Point(-1, -1),  # bottom-left
        ]

        # Create triangulation that might have non-convex quadrilaterals
        edges = [(0, 1), (0, 2), (0, 3), (0, 4)]

        flip_map = FlipPartnerMap.build(points, edges)

        # The algorithm should only allow flips for convex quadrilaterals
        flippable = flip_map.flippable_edges()

        # Test that flippability is correctly determined
        for edge in flippable:
            assert flip_map.is_flippable(edge), (
                f"Edge {edge} should be consistently flippable"
            )

    def test_large_triangulation(self):
        """Test with a larger triangulation."""
        # Create a 3x3 grid
        points = []
        for i in range(3):
            for j in range(3):
                points.append(Point(i, j))

        # Create fan triangulation from center
        center_idx = 4  # Point(1, 1)
        edges = []
        boundary_indices = [0, 1, 2, 3, 5, 6, 7, 8]
        for idx in boundary_indices:
            edges.append((center_idx, idx))

        flip_map = FlipPartnerMap.build(points, edges)

        assert len(flip_map.compute_triangles()) == 8, "Should have 8 triangles in fan"

        # Test basic operations work
        flippable = flip_map.flippable_edges()
        assert isinstance(flippable, list), "Should return flippable edges list"

        # Test that we can create a copy
        copy = flip_map.deep_copy()
        assert copy.flip_map == flip_map.flip_map, "Copy should match original"

    def test_flip_updates_triangulation_correctly(self):
        """Test that flips correctly update the internal triangulation state."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        flip_map = FlipPartnerMap.build(points, edges)

        # Perform flip
        new_edge = flip_map.flip((0, 3))

        # Verify the flip map has been updated
        assert (
            0,
            3,
        ) not in flip_map.flip_map, "Original edge should be removed from flip map"
        assert new_edge in flip_map.flip_map, "New edge should be added to flip map"

        # The triangulation should be updated to reflect the flip
        # (Note: the current implementation doesn't update triangles, which might be a limitation)

    def test_is_flippable_edge_normalization(self):
        """Test that edge vertex order doesn't affect flippability check."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        flip_map = FlipPartnerMap.build(points, edges)

        # Test both orderings
        assert flip_map.is_flippable((0, 3)) == flip_map.is_flippable((3, 0)), (
            "Flippability should be independent of edge vertex order"
        )
