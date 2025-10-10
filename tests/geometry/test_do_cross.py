"""
Unit tests for the do_cross function.

Tests verify that the function correctly identifies when two segments
properly cross each other (intersect at a point that is not an endpoint
of either segment).
"""

from cgshop2026_pyutils.geometry import do_cross, Segment, Point


class TestDoCross:
    """Test suite for the do_cross function."""

    def test_crossing_segments(self):
        """Test segments that properly cross each other."""
        # X-shaped crossing
        s1 = Segment(Point(0, 0), Point(2, 2))  # Diagonal from bottom-left to top-right
        s2 = Segment(Point(0, 2), Point(2, 0))  # Diagonal from top-left to bottom-right

        assert do_cross(s1, s2), "X-shaped segments should cross"

    def test_crossing_segments_reverse_order(self):
        """Test that segment order doesn't matter."""
        s1 = Segment(Point(0, 0), Point(2, 2))
        s2 = Segment(Point(0, 2), Point(2, 0))

        # Test both orders
        assert do_cross(s1, s2) == do_cross(s2, s1), "Crossing should be symmetric"
        assert do_cross(s1, s2), "Segments should cross regardless of order"

    def test_horizontal_vertical_crossing(self):
        """Test horizontal and vertical segments crossing."""
        horizontal = Segment(
            Point(-1, 0), Point(1, 0)
        )  # Horizontal line through origin
        vertical = Segment(Point(0, -1), Point(0, 1))  # Vertical line through origin

        assert do_cross(horizontal, vertical), (
            "Horizontal and vertical segments should cross"
        )

    def test_parallel_segments_no_crossing(self):
        """Test that parallel segments don't cross."""
        s1 = Segment(Point(0, 0), Point(2, 0))  # Horizontal line
        s2 = Segment(Point(0, 1), Point(2, 1))  # Parallel horizontal line

        assert not do_cross(s1, s2), "Parallel segments should not cross"

    def test_collinear_segments_no_crossing(self):
        """Test that collinear segments don't cross."""
        s1 = Segment(Point(0, 0), Point(2, 0))  # Line segment
        s2 = Segment(Point(1, 0), Point(3, 0))  # Overlapping collinear segment

        assert not do_cross(s1, s2), "Collinear segments should not cross"

    def test_segments_sharing_endpoint_no_crossing(self):
        """Test that segments sharing an endpoint don't cross."""
        shared_point = Point(1, 1)
        s1 = Segment(Point(0, 0), shared_point)
        s2 = Segment(shared_point, Point(2, 2))

        assert not do_cross(s1, s2), "Segments sharing endpoint should not cross"

    def test_segments_with_endpoint_on_other_segment_no_crossing(self):
        """Test that segments where one endpoint lies on the other segment don't cross."""
        s1 = Segment(Point(0, 0), Point(2, 0))  # Horizontal line
        s2 = Segment(Point(1, 0), Point(1, 1))  # Vertical segment starting on s1

        assert not do_cross(s1, s2), (
            "Segment with endpoint on other segment should not cross"
        )

    def test_intersecting_at_endpoint_no_crossing(self):
        """Test various cases where segments intersect at endpoints."""
        # T-junction case
        base = Segment(Point(0, 0), Point(2, 0))
        perpendicular = Segment(Point(1, 0), Point(1, 1))

        assert not do_cross(base, perpendicular), (
            "T-junction should not be considered crossing"
        )

        # L-shape case
        horizontal = Segment(Point(0, 0), Point(1, 0))
        vertical = Segment(Point(1, 0), Point(1, 1))

        assert not do_cross(horizontal, vertical), (
            "L-shape should not be considered crossing"
        )

    def test_non_intersecting_segments(self):
        """Test segments that don't intersect at all."""
        s1 = Segment(Point(0, 0), Point(1, 1))  # Bottom-left diagonal
        s2 = Segment(Point(2, 0), Point(3, 1))  # Top-right diagonal, separate

        assert not do_cross(s1, s2), "Non-intersecting segments should not cross"

    def test_segments_that_would_intersect_if_extended(self):
        """Test segments that would intersect if extended but don't actually intersect."""
        # These segments don't actually intersect within their bounds
        s1_short = Segment(Point(0, 0), Point(0.5, 0.5))
        s2_separate = Segment(Point(0, 2), Point(2, 0))

        assert not do_cross(s1_short, s2_separate), (
            "Segments that don't actually intersect should not cross"
        )

    def test_crossing_with_rational_coordinates(self):
        """Test crossing with non-integer coordinates."""
        s1 = Segment(Point(0.5, 0), Point(1.5, 2))
        s2 = Segment(Point(0, 1), Point(2, 1))

        assert do_cross(s1, s2), (
            "Segments with rational coordinates should cross correctly"
        )

    def test_crossing_at_exact_midpoint(self):
        """Test segments that cross at their midpoints."""
        s1 = Segment(Point(-1, 0), Point(1, 0))  # Horizontal through origin
        s2 = Segment(Point(0, -1), Point(0, 1))  # Vertical through origin

        assert do_cross(s1, s2), "Segments crossing at midpoints should be detected"

    def test_almost_parallel_segments_crossing(self):
        """Test nearly parallel segments that still cross."""
        s1 = Segment(Point(0, 0), Point(10, 1))  # Nearly horizontal
        s2 = Segment(Point(0, 1), Point(10, 0))  # Nearly horizontal, opposite slope

        assert do_cross(s1, s2), "Nearly parallel crossing segments should be detected"

    def test_identical_segments_no_crossing(self):
        """Test that identical segments don't cross."""
        s1 = Segment(Point(0, 0), Point(1, 1))
        s2 = Segment(Point(0, 0), Point(1, 1))  # Identical segment

        assert not do_cross(s1, s2), "Identical segments should not cross"

    def test_reversed_identical_segments_no_crossing(self):
        """Test that reversed identical segments don't cross."""
        s1 = Segment(Point(0, 0), Point(1, 1))
        s2 = Segment(Point(1, 1), Point(0, 0))  # Same segment, reversed

        assert not do_cross(s1, s2), "Reversed identical segments should not cross"

    def test_zero_length_segments(self):
        """Test behavior with zero-length segments."""
        point_segment = Segment(Point(1, 1), Point(1, 1))  # Zero length
        normal_segment = Segment(Point(0, 0), Point(2, 2))

        assert not do_cross(point_segment, normal_segment), (
            "Zero-length segment should not cross"
        )

    def test_crossing_with_negative_coordinates(self):
        """Test crossing with negative coordinates."""
        s1 = Segment(Point(-2, -2), Point(2, 2))  # Diagonal through origin
        s2 = Segment(Point(-2, 2), Point(2, -2))  # Opposite diagonal

        assert do_cross(s1, s2), (
            "Segments with negative coordinates should cross correctly"
        )

    def test_complex_crossing_scenario(self):
        """Test a more complex crossing scenario."""
        # Create segments that cross at a non-trivial point
        s1 = Segment(Point(1, 0), Point(3, 4))  # Steep positive slope
        s2 = Segment(Point(0, 3), Point(4, 1))  # Negative slope

        assert do_cross(s1, s2), "Complex crossing scenario should be detected"

    def test_barely_touching_endpoints(self):
        """Test segments where endpoints barely touch."""
        s1 = Segment(Point(0, 0), Point(1, 0))
        s2 = Segment(Point(1, 0), Point(2, 1))  # Shares endpoint (1,0)

        assert not do_cross(s1, s2), "Segments touching at endpoints should not cross"

    def test_crossing_with_large_coordinates(self):
        """Test crossing with large coordinate values."""
        s1 = Segment(Point(0, 0), Point(1000, 1000))
        s2 = Segment(Point(0, 1000), Point(1000, 0))

        assert do_cross(s1, s2), (
            "Segments with large coordinates should cross correctly"
        )

    def test_very_close_parallel_segments(self):
        """Test very close parallel segments that don't cross."""
        s1 = Segment(Point(0, 0), Point(10, 0))
        s2 = Segment(Point(0, 0.0001), Point(10, 0.0001))  # Very close parallel

        assert not do_cross(s1, s2), "Very close parallel segments should not cross"

    def test_segments_meeting_at_multiple_points_collinear(self):
        """Test overlapping collinear segments."""
        s1 = Segment(Point(0, 0), Point(4, 0))
        s2 = Segment(Point(2, 0), Point(6, 0))  # Overlapping on same line

        assert not do_cross(s1, s2), "Overlapping collinear segments should not cross"

    def test_perpendicular_segments_crossing(self):
        """Test perpendicular segments that cross."""
        s1 = Segment(Point(0, 1), Point(2, 1))  # Horizontal
        s2 = Segment(Point(1, 0), Point(1, 2))  # Vertical, crossing s1

        assert do_cross(s1, s2), "Perpendicular crossing segments should be detected"
