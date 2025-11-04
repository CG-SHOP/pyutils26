#pragma once

#include "cgal_types.h"
#include <optional>
#include <tuple>
#include <vector>

namespace cgshop2026 {

/**
 * Two segments cross if they intersect in a point that is not an endpoint.
 * No endpoint is allowed to lie on the other segment.
 */
bool do_cross(const Segment2 &s1, const Segment2 &s2);

/**
 * This function checks if the given set of edges forms a triangulation of the
 * provided points. It uses the CGAL arrangement data structure to insert the
 * edges and verify the triangulation properties.
 */
bool is_triangulation(const std::vector<Point> &points,
                      const std::vector<std::tuple<int, int>> &edges,
                      bool verbose = false);

/**
 * This function computes all triangles formed by the given set of points and
 * edges. It returns a list of triangles, where each triangle is represented by
 * a tuple of three point indices. Edges that appear only once will be on the
 * convex hull. Otherwise, all edges should appear exactly twice. The indices
 * will be sorted in each triangle, and the list of triangles will also be
 * sorted.
 */
std::vector<std::tuple<int, int, int>>
compute_triangles(const std::vector<Point>& points,
                  const std::vector<std::tuple<int, int>>& edges);

} // namespace cgshop2026
