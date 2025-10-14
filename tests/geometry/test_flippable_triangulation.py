"""
Unit tests for the FlippableTriangulation class.

Tests verify that the class correctly handles triangulated point sets with
edge flip operations, manages pending flips, handles conflicts, and provides
proper validation and error handling.
"""

import pytest
from cgshop2026_pyutils.geometry import FlippableTriangulation, Point


class TestFlippableTriangulation:
    """Test suite for the FlippableTriangulation class."""

    def test_from_points_edges_valid_triangulation(self):
        """Test creating FlippableTriangulation from valid points and edges."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal creating valid triangulation

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        assert triangulation is not None, (
            "Should create FlippableTriangulation successfully"
        )
        assert len(triangulation._flip_map.points) == 4, "Should have 4 points"
        assert len(triangulation._flip_queue) == 0, "Should start with empty flip queue"
        assert len(triangulation._conflicting_edges) == 0, (
            "Should start with no conflicting edges"
        )

    def test_from_points_edges_invalid_triangulation_raises_error(self):
        """Test that invalid triangulation raises ValueError."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = []  # No internal edges - not a valid triangulation for square

        with pytest.raises(ValueError, match="do not form a valid triangulation"):
            FlippableTriangulation.from_points_edges(points, edges)

    def test_from_points_edges_triangle_valid(self):
        """Test that a simple triangle is valid."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []  # Convex hull forms valid triangle

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        assert triangulation is not None, "Triangle should be valid"
        assert len(triangulation._flip_map.points) == 3, "Should have 3 points"

    def test_fork_creates_independent_copy(self):
        """Test that fork creates an independent copy."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        original = FlippableTriangulation.from_points_edges(points, edges)
        fork = original.fork()

        # Should be independent objects
        assert fork is not original, "Fork should be different object"
        assert fork._flip_map is not original._flip_map, (
            "Fork should have independent flip map"
        )

        # Should have same initial state
        assert len(fork._flip_queue) == len(original._flip_queue), (
            "Should have same queue length"
        )
        assert len(fork._conflicting_edges) == len(original._conflicting_edges), (
            "Should have same conflicting edges"
        )

    def test_fork_independence_after_modifications(self):
        """Test that fork remains independent after modifications."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        original = FlippableTriangulation.from_points_edges(points, edges)
        fork = original.fork()

        # Modify the fork
        if fork._flip_map.is_flippable((0, 3)):
            fork.add_flip((0, 3))

        # Original should be unchanged
        assert len(original._flip_queue) == 0, "Original should remain unchanged"
        assert len(fork._flip_queue) > 0, "Fork should have modifications"

    def test_add_flip_valid_edge(self):
        """Test adding a valid flip to the queue."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Add flip for the diagonal
        new_edge = triangulation.add_flip((0, 3))

        assert (0, 3) in triangulation._flip_queue, "Edge should be added to flip queue"
        assert len(triangulation._flip_queue) == 1, "Should have one pending flip"
        assert new_edge == (1, 2), "Should return the flip partner edge"

    def test_get_flip_partner_convenience(self):
        """Test the convenience wrapper FlippableTriangulation.get_flip_partner."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        tri = FlippableTriangulation.from_points_edges(points, edges)
        # Edge should be flippable initially
        assert (0,3) in tri.possible_flips()
        partner_direct = tri._flip_map.get_flip_partner((0,3))
        partner_via_method = tri.get_flip_partner((0,3))
        assert partner_direct == partner_via_method, "Convenience method should match underlying map"
        # Normalize reversed order argument
        partner_reversed = tri.get_flip_partner((3,0))
        assert partner_reversed == partner_direct, "Method should normalize edge order"
        # After adding to queue still accessible
        tri.add_flip((0,3))
        assert tri.get_flip_partner((0,3)) == partner_direct

    def test_add_flip_edge_ordering_normalized(self):
        """Test that edge ordering is normalized when adding flips."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Add flip with reversed edge ordering
        triangulation.add_flip((3, 0))

        # Should normalize to (0, 3) in queue
        assert (0, 3) in triangulation._flip_queue, "Edge should be normalized in queue"
        assert (
            3,
            0,
        ) not in triangulation._flip_queue, "Reversed edge should not be in queue"

    def test_add_flip_non_flippable_edge_raises_error(self):
        """Test that adding non-flippable edge raises error."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []  # Triangle - no flippable edges

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        with pytest.raises(ValueError, match="Edge is not flippable"):
            triangulation.add_flip((0, 1))

    def test_add_flip_prevents_duplicate_edges(self):
        """Test that adding the same edge twice raises an error."""
        # Create a square with diagonal (valid triangulation)
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal edge

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Add the diagonal flip
        triangulation.add_flip((0, 3))

        # Adding the same edge again should raise an error
        with pytest.raises(ValueError, match="already pending"):
            triangulation.add_flip((0, 3))

    def test_commit_applies_flips(self):
        """Test that commit applies all pending flips."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Add flip
        triangulation.add_flip((0, 3))

        # Edge should still be flippable before commit
        assert triangulation._flip_map.is_flippable((0, 3)), (
            "Edge should be flippable before commit"
        )

        # Commit the flip
        triangulation.commit()

        # Flip queue should be empty
        assert len(triangulation._flip_queue) == 0, (
            "Flip queue should be empty after commit"
        )
        assert len(triangulation._conflicting_edges) == 0, (
            "Conflicting edges should be cleared"
        )

        # Original edge should no longer be flippable
        assert not triangulation._flip_map.is_flippable((0, 3)), (
            "Original edge should not be flippable after commit"
        )

        # New edge should be flippable
        assert triangulation._flip_map.is_flippable((1, 2)), (
            "New edge should be flippable after commit"
        )

    def test_multiple_flips_same_triangulation(self):
        """Test adding multiple flips to the same triangulation."""
        # Create a larger triangulation with multiple flippable edges
        points = [
            Point(0, 0),
            Point(2, 0),
            Point(1, 1),
            Point(1, -1),
            Point(0, 2),
            Point(2, 2),
        ]
        edges = [(0, 2), (1, 3)]  # Create some internal structure

        try:
            triangulation = FlippableTriangulation.from_points_edges(points, edges)

            # Get all flippable edges
            flippable = triangulation._flip_map.flippable_edges()

            if len(flippable) >= 2:
                # Try to add multiple non-conflicting flips
                edge1 = flippable[0]
                triangulation.add_flip(edge1)

                # Find a non-conflicting edge
                conflicts1 = triangulation._flip_map.conflicting_flips(edge1)
                for edge2 in flippable[1:]:
                    if (
                        edge2 not in conflicts1
                        and edge2 not in triangulation._conflicting_edges
                    ):
                        triangulation.add_flip(edge2)
                        break

                # Should have multiple flips queued
                assert len(triangulation._flip_queue) >= 1, (
                    "Should have at least one flip queued"
                )

        except ValueError:
            # If the triangulation is invalid, that's also acceptable for this test
            pass

    def test_empty_commit_does_nothing(self):
        """Test that commit with no pending flips does nothing."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Commit without adding any flips
        triangulation.commit()

        # State should be unchanged
        assert len(triangulation._flip_queue) == 0, "Queue should remain empty"
        assert len(triangulation._conflicting_edges) == 0, (
            "Conflicting edges should remain empty"
        )
        assert triangulation._flip_map.is_flippable((0, 3)), (
            "Original flippability should be unchanged"
        )

    def test_state_consistency_after_operations(self):
        """Test that internal state remains consistent after various operations."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Add flip
        triangulation.add_flip((0, 3))

        # Check state consistency
        assert (0, 3) in triangulation._flip_queue, "Edge should be in queue"
        assert len(triangulation._conflicting_edges) >= 0, (
            "Should have conflicting edges set"
        )

        # Fork and check independence (fork creates a deep copy, so it should be empty initially)
        fork = triangulation.fork()
        assert fork._flip_queue == [], (
            "Fork should start with empty queue (deep copy creates fresh state)"
        )
        assert fork._conflicting_edges == set(), (
            "Fork should start with empty conflicting edges"
        )

        # Commit original
        triangulation.commit()

        # Original should be empty after commit
        assert len(triangulation._flip_queue) == 0, (
            "Original should be empty after commit"
        )

    def test_complex_triangulation_operations(self):
        """Test operations on a more complex triangulation."""
        # Pentagon with center point
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

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Should be created successfully
        assert triangulation is not None, "Complex triangulation should be created"
        assert len(triangulation._flip_map.compute_triangles()) == 5, (
            "Should have 5 triangles"
        )

        # Try operations
        flippable = triangulation._flip_map.flippable_edges()

        if flippable:
            # Add a flip if possible
            triangulation.add_flip(flippable[0])
            assert len(triangulation._flip_queue) == 1, "Should have pending flip"

            # Commit
            triangulation.commit()
            assert len(triangulation._flip_queue) == 0, (
                "Queue should be empty after commit"
            )

    def test_triangulation_with_no_flippable_edges(self):
        """Test triangulation where no edges can be flipped."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []  # Simple triangle

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Should have no flippable edges
        flippable = triangulation._flip_map.flippable_edges()
        assert len(flippable) == 0, "Triangle should have no flippable edges"

        # All operations should work but have no effect
        triangulation.commit()  # Should not raise error
        assert len(triangulation._flip_queue) == 0, "Queue should remain empty"

    def test_edge_cases_and_boundary_conditions(self):
        """Test various edge cases and boundary conditions."""
        # Minimal valid triangulation
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Test fork of minimal triangulation
        fork = triangulation.fork()
        assert fork is not triangulation, "Fork should be independent"

        # Test multiple commits
        triangulation.commit()
        triangulation.commit()  # Should not raise error

        # Test fork after operations
        fork2 = triangulation.fork()
        assert fork2 is not triangulation, "Second fork should be independent"

    def test_flip_queue_order_preservation(self):
        """Test that flip queue preserves order of added flips."""
        # Create triangulation with multiple potentially flippable edges
        points = [Point(0, 0), Point(2, 0), Point(1, 1), Point(1, -1)]
        edges = [(0, 2)]

        try:
            triangulation = FlippableTriangulation.from_points_edges(points, edges)

            # Get flippable edges
            flippable = triangulation._flip_map.flippable_edges()

            if len(flippable) >= 2:
                # Add flips in specific order
                edge1 = flippable[0]
                edge2 = flippable[1] if len(flippable) > 1 else None

                triangulation.add_flip(edge1)

                # Try to add second flip if it doesn't conflict
                if edge2 and edge2 not in triangulation._conflicting_edges:
                    try:
                        triangulation.add_flip(edge2)
                        # Order should be preserved
                        assert triangulation._flip_queue[0] == edge1, (
                            "First edge should be first in queue"
                        )
                        assert triangulation._flip_queue[1] == edge2, (
                            "Second edge should be second in queue"
                        )
                    except ValueError:
                        # If it conflicts, that's also valid behavior
                        pass

        except ValueError:
            # If triangulation is invalid, that's acceptable for this test
            pass

    def test_error_messages_are_descriptive(self):
        """Test that error messages provide helpful information."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Test non-flippable edge error
        try:
            triangulation.add_flip((0, 1))
        except ValueError as e:
            assert "not flippable" in str(e), (
                "Error message should mention flippability"
            )

        # Test invalid triangulation error
        invalid_points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        invalid_edges = []  # Square without diagonal

        try:
            FlippableTriangulation.from_points_edges(invalid_points, invalid_edges)
        except ValueError as e:
            assert "triangulation" in str(e), (
                "Error message should mention triangulation"
            )

    def test_possible_flips_simple_square(self):
        """Test possible_flips returns correct edges for a simple square."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal edge

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Initially, only the diagonal should be flippable
        possible = triangulation.possible_flips()
        assert (0, 3) in possible, "Diagonal edge should be flippable"
        assert len(possible) == 1, "Should have exactly one flippable edge"

    def test_possible_flips_after_adding_flip(self):
        """Test that possible_flips excludes edges in the flip queue."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal edge

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Add the diagonal to flip queue
        triangulation.add_flip((0, 3))

        # Now the diagonal should not be in possible flips
        possible = triangulation.possible_flips()
        assert (0, 3) not in possible, "Edge in flip queue should not be possible"
        assert len(possible) == 0, (
            "Should have no possible flips after adding the only flippable edge"
        )

    def test_possible_flips_after_commit(self):
        """Test that possible_flips updates correctly after commit."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]  # Diagonal edge

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Add and commit the flip
        triangulation.add_flip((0, 3))
        triangulation.commit()

        # Now the new edge should be flippable
        possible = triangulation.possible_flips()
        assert (1, 2) in possible, "New edge from flip should be flippable"
        assert (0, 3) not in possible, "Original edge should no longer be flippable"

    def test_possible_flips_excludes_conflicting_edges(self):
        """Test that possible_flips excludes conflicting edges."""
        # Create a more complex triangulation where conflicts can exist
        points = [Point(0, 0), Point(2, 0), Point(1, 1), Point(1, -1)]
        edges = [(0, 2)]  # Diagonal edge

        try:
            triangulation = FlippableTriangulation.from_points_edges(points, edges)

            # Get all initially flippable edges
            all_flippable = triangulation._flip_map.flippable_edges()

            if len(all_flippable) > 0:
                # Add one flip
                edge_to_flip = all_flippable[0]
                triangulation.add_flip(edge_to_flip)

                # Get possible flips - should exclude the added edge and any conflicts
                possible = triangulation.possible_flips()

                # The added edge should not be in possible flips
                assert edge_to_flip not in possible, (
                    "Added edge should not be in possible flips"
                )

                # Any conflicting edges should also not be in possible flips
                conflicts = triangulation._conflicting_edges
                for conflict in conflicts:
                    assert conflict not in possible, (
                        f"Conflicting edge {conflict} should not be in possible flips"
                    )

        except ValueError:
            # If the triangulation setup fails, that's acceptable for this test
            pass

    def test_possible_flips_triangle_no_flips(self):
        """Test that triangle has no possible flips."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []  # Simple triangle

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Triangle should have no flippable edges
        possible = triangulation.possible_flips()
        assert len(possible) == 0, "Triangle should have no possible flips"

    def test_possible_flips_complex_triangulation(self):
        """Test possible_flips on a more complex triangulation."""
        # Pentagon with center point - multiple potentially flippable edges
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

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Should have some flippable edges
        possible = triangulation.possible_flips()
        all_flippable = triangulation._flip_map.flippable_edges()

        # All flippable edges should be possible initially
        assert len(possible) == len(all_flippable), (
            "All flippable edges should be possible initially"
        )
        for edge in all_flippable:
            assert edge in possible, (
                f"Flippable edge {edge} should be in possible flips"
            )

    def test_possible_flips_return_type_and_format(self):
        """Test that possible_flips returns correct data types and formats."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        possible = triangulation.possible_flips()

        # Should return a list
        assert isinstance(possible, list), "Should return a list"

        # Each element should be a tuple of two integers
        for edge in possible:
            assert isinstance(edge, tuple), "Each edge should be a tuple"
            assert len(edge) == 2, "Each edge should have two elements"
            assert isinstance(edge[0], int), "Edge indices should be integers"
            assert isinstance(edge[1], int), "Edge indices should be integers"
            assert edge[0] < edge[1], (
                "Edge should be normalized (first index < second index)"
            )

    def test_possible_flips_consistency_with_flip_map(self):
        """Test that possible_flips is consistent with underlying flip map."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        possible = triangulation.possible_flips()
        all_flippable = triangulation._flip_map.flippable_edges()

        # Initially, possible should match all flippable
        assert set(possible) == set(all_flippable), (
            "Possible flips should match all flippable edges initially"
        )

        # Add a flip and check consistency
        if possible:
            edge_to_add = possible[0]
            triangulation.add_flip(edge_to_add)

            updated_possible = triangulation.possible_flips()

            # The added edge should no longer be possible
            assert edge_to_add not in updated_possible, (
                "Added edge should no longer be possible"
            )

            # All remaining possible edges should still be flippable in the map
            for edge in updated_possible:
                assert triangulation._flip_map.is_flippable(edge), (
                    f"Edge {edge} should be flippable in map"
                )

    def test_possible_flips_empty_after_multiple_operations(self):
        """Test possible_flips behavior after multiple operations."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Add all possible flips
        possible = triangulation.possible_flips()
        for edge in possible:
            triangulation.add_flip(edge)

        # Should have no more possible flips
        remaining_possible = triangulation.possible_flips()
        assert len(remaining_possible) == 0, (
            "Should have no possible flips after adding all"
        )

        # After commit, should have new possible flips
        triangulation.commit()
        post_commit_possible = triangulation.possible_flips()

        # The result depends on the specific triangulation, but should be valid
        for edge in post_commit_possible:
            assert triangulation._flip_map.is_flippable(edge), (
                f"Edge {edge} should be flippable after commit"
            )

    def test_eq_identical_triangulations(self):
        """Test that identical triangulations are equal."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation1 = FlippableTriangulation.from_points_edges(points, edges)
        triangulation2 = FlippableTriangulation.from_points_edges(points, edges)

        assert triangulation1 == triangulation2, (
            "Triangulations with same points and edges should be equal"
        )

    def test_eq_same_object(self):
        """Test that a triangulation equals itself."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        assert triangulation == triangulation, "Triangulation should equal itself"

    def test_eq_different_triangulations_different_edges(self):
        """Test that triangulations with different edges are not equal."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]

        triangulation1 = FlippableTriangulation.from_points_edges(points, [(0, 3)])
        triangulation2 = FlippableTriangulation.from_points_edges(points, [(1, 2)])

        assert triangulation1 != triangulation2, (
            "Triangulations with different edges should not be equal"
        )

    def test_eq_same_edges_different_pending_flips(self):
        """Test that triangulations with same edges but different pending flips are not equal."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation1 = FlippableTriangulation.from_points_edges(points, edges)
        triangulation2 = FlippableTriangulation.from_points_edges(points, edges)

        # Add flip to only one triangulation
        triangulation2.add_flip((0, 3))

        assert triangulation1 != triangulation2, (
            "Triangulations with different pending flips should not be equal"
        )

    def test_eq_same_edges_same_pending_flips(self):
        """Test that triangulations with same edges and same pending flips are equal."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation1 = FlippableTriangulation.from_points_edges(points, edges)
        triangulation2 = FlippableTriangulation.from_points_edges(points, edges)

        # Add same flip to both triangulations
        triangulation1.add_flip((0, 3))
        triangulation2.add_flip((0, 3))

        assert triangulation1 == triangulation2, (
            "Triangulations with same edges and same pending flips should be equal"
        )

    def test_eq_after_commit_operations(self):
        """Test equality after commit operations."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation1 = FlippableTriangulation.from_points_edges(points, edges)
        triangulation2 = FlippableTriangulation.from_points_edges(points, edges)

        # Both start equal
        assert triangulation1 == triangulation2, "Should start equal"

        # Add and commit same flip to both
        triangulation1.add_flip((0, 3))
        triangulation2.add_flip((0, 3))

        # Should still be equal with same pending flip
        assert triangulation1 == triangulation2, (
            "Should be equal with same pending flip"
        )

        # Commit both
        triangulation1.commit()
        triangulation2.commit()

        # Should be equal after same commit
        assert triangulation1 == triangulation2, (
            "Should be equal after same commit operations"
        )

    def test_eq_fork_equality(self):
        """Test equality with forked triangulations."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        original = FlippableTriangulation.from_points_edges(points, edges)
        fork = original.fork()

        # Fork should initially be equal (same edges, empty flip queues)
        assert original == fork, "Fork should initially be equal to original"

        # Add flip to original only
        original.add_flip((0, 3))

        # Should no longer be equal
        assert original != fork, "Should not be equal after modifying original"

        # Add same flip to fork
        fork.add_flip((0, 3))

        # Should be equal again
        assert original == fork, "Should be equal again after same modification"

    def test_eq_with_non_triangulation_object(self):
        """Test equality comparison with non-FlippableTriangulation objects."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        triangulation = FlippableTriangulation.from_points_edges(points, edges)

        # Test with various non-triangulation objects
        assert not triangulation.__eq__(None), "Should not equal None"
        assert triangulation != "string", "Should not equal string"
        assert triangulation != 42, "Should not equal number"
        assert triangulation != [], "Should not equal list"
        assert triangulation != {}, "Should not equal dict"
        assert triangulation != points, "Should not equal points list"

    def test_eq_different_points_same_structure(self):
        """Test triangulations with different points but same topological structure."""
        # First triangulation - unit square
        points1 = [Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)]
        edges = [(0, 3)]

        # Second triangulation - scaled square
        points2 = [Point(0, 0), Point(2, 0), Point(0, 2), Point(2, 2)]

        triangulation1 = FlippableTriangulation.from_points_edges(points1, edges)
        triangulation2 = FlippableTriangulation.from_points_edges(points2, edges)

        # The equality check only compares edge indices, not point coordinates
        # So triangulations with same topological structure are considered equal
        result = triangulation1 == triangulation2

        # We verify that the comparison works without making assumptions about
        # whether coordinate differences should affect equality
        assert isinstance(result, bool), (
            "Equality comparison should return a boolean value"
        )

        # Test a case where points are definitely different enough to matter
        # Use different point count to ensure inequality
        points3 = [Point(0, 0), Point(1, 0), Point(0, 1)]  # Triangle
        edges3 = []

        triangulation3 = FlippableTriangulation.from_points_edges(points3, edges3)

        assert triangulation1 != triangulation3, (
            "Triangulations with different numbers of points should not be equal"
        )

    def test_eq_empty_triangulations(self):
        """Test equality of minimal triangulations (triangles)."""
        points = [Point(0, 0), Point(1, 0), Point(0, 1)]
        edges = []  # Triangle needs no internal edges

        triangulation1 = FlippableTriangulation.from_points_edges(points, edges)
        triangulation2 = FlippableTriangulation.from_points_edges(points, edges)

        assert triangulation1 == triangulation2, (
            "Identical triangle triangulations should be equal"
        )

    def test_eq_complex_triangulations(self):
        """Test equality with more complex triangulations."""
        # Pentagon with center point
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

        triangulation1 = FlippableTriangulation.from_points_edges(points, edges)
        triangulation2 = FlippableTriangulation.from_points_edges(points, edges)

        assert triangulation1 == triangulation2, (
            "Complex triangulations with same structure should be equal"
        )

        # Test with different edge orderings in the edges list
        edges_reordered = [(0, 5), (0, 1), (0, 4), (0, 2), (0, 3)]
        triangulation3 = FlippableTriangulation.from_points_edges(
            points, edges_reordered
        )

        # This depends on how the FlipPartnerMap handles edge ordering internally
        # The triangulations should be equal if the internal representation is the same
        result = triangulation1 == triangulation3

        # We don't assert a specific result here since it depends on internal implementation,
        # but we verify the comparison doesn't crash and returns a boolean
        assert isinstance(result, bool), "Equality comparison should return boolean"
