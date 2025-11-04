#pragma once

#include "cgal_types.h"
#include <map>
#include <optional>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace cgshop2026 {

/**
 * Build a point-to-index mapping and check for duplicate points.
 * Returns empty optional if duplicates are found.
 */
std::optional<std::map<Point, int, LessPointXY>>
build_point_index_map(const std::vector<Point> &points, bool verbose);

/**
 * Insert edges into the arrangement and validate each insertion.
 */
bool insert_edges_into_arrangement(
    const std::vector<Point> &points,
    const std::vector<std::tuple<int, int>> &edges,
    Arrangement_2 &arrangement,
    PointLocation &point_location,
    bool verbose);

/**
 * Add convex hull edges to the arrangement if not already present.
 */
void add_convex_hull_to_arrangement(
    const std::vector<Point> &points,
    Arrangement_2 &arrangement,
    PointLocation &point_location,
    bool verbose);

/**
 * Verify that the arrangement has exactly the expected number of vertices
 * (no new intersections created, no points missing).
 */
bool validate_vertex_count(
    const Arrangement_2 &arrangement,
    size_t expected_count,
    const std::vector<Point> &points,
    bool verbose);

/**
 * Validate that all faces in the arrangement are triangles.
 * Collects all edges from triangular faces for later validation.
 */
bool validate_all_faces_triangular(
    const Arrangement_2 &arrangement,
    const std::map<Point, int, LessPointXY> &idx_of,
    std::unordered_set<std::tuple<int, int>, TupleHash> &edges_in_arrangement,
    bool verbose);

/**
 * Verify that all input edges appear in the arrangement faces.
 */
bool validate_input_edges_present(
    const std::vector<std::tuple<int, int>> &edges,
    const std::unordered_set<std::tuple<int, int>, TupleHash> &edges_in_arrangement,
    bool verbose);

} // namespace cgshop2026
