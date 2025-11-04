#include "geometry_operations.h"
#include "triangulation_validation.h"
#include <CGAL/convex_hull_2.h>
#include <algorithm>
#include <array>
#include <iostream>

namespace cgshop2026 {

/**
 * Two segments cross if they intersect in a point that is not an endpoint.
 * No endpoint is allowed to lie on the other segment.
 */
bool do_cross(const Segment2 &s1, const Segment2 &s2) {
  auto result = CGAL::intersection(s1, s2);
  if (result) {
    if (const Point *p = std::get_if<Point>(&*result)) {
      // Check if the intersection point is an endpoint of either segment
      if (*p == s1.source() || *p == s1.target() || *p == s2.source() ||
          *p == s2.target()) {
        return false; // Intersection at an endpoint, not a crossing
      }
      return true; // Proper crossing
    }
  }
  return false; // No intersection
}

// ============================================================================
// is_triangulation - Main validation function
// ============================================================================

/**
 * This function checks if the given set of edges forms a triangulation of the
 * provided points. It uses the CGAL arrangement data structure to insert the
 * edges and verify the triangulation properties.
 */
bool is_triangulation(const std::vector<Point> &points,
                      const std::vector<std::tuple<int, int>> &edges,
                      bool verbose) {
  if (verbose) {
    fmt::print("Validating triangulation with {} points and {} edges.\n",
               points.size(), edges.size());
  }

  // Step 1: Build point-to-index mapping and check for duplicates
  const auto idx_of_opt = build_point_index_map(points, verbose);
  if (!idx_of_opt) {
    return false;
  }
  const auto &idx_of = *idx_of_opt;

  // Step 2: Create arrangement and insert edges
  Arrangement_2 arrangement;
  PointLocation point_location(arrangement);

  if (!insert_edges_into_arrangement(points, edges, arrangement,
                                      point_location, verbose)) {
    return false;
  }

  // Step 3: Add convex hull edges
  add_convex_hull_to_arrangement(points, arrangement, point_location, verbose);

  // Step 4: Validate vertex count (no new intersections, no missing points)
  if (!validate_vertex_count(arrangement, points.size(), points, verbose)) {
    return false;
  }

  // Step 5: Validate all faces are triangular and collect edges
  // Reserve space: triangulation of n points has ~3n edges (Euler's formula)
  std::unordered_set<std::tuple<int, int>, TupleHash> edges_in_arrangement;
  edges_in_arrangement.reserve(3 * points.size());

  if (!validate_all_faces_triangular(arrangement, idx_of,
                                       edges_in_arrangement, verbose)) {
    return false;
  }

  // Step 6: Verify all input edges appear in the arrangement
  if (!validate_input_edges_present(edges, edges_in_arrangement, verbose)) {
    return false;
  }

  // Success
  if (verbose) {
    fmt::print("Triangulation validation complete: Valid triangulation\n");
  }

  return true;
}

// ============================================================================
// Helper functions for compute_triangles
// ============================================================================

/**
 * Build arrangement from points and edges, including convex hull.
 */
static void build_arrangement_for_triangles(
    const std::vector<Point> &points,
    const std::vector<std::tuple<int, int>> &edges,
    Arrangement_2 &arrangement,
    PointLocation &point_location) {

  // Insert input edges
  for (const auto &edge : edges) {
    const int i = std::get<0>(edge);
    const int j = std::get<1>(edge);
    if (i < 0 || i >= static_cast<int>(points.size()) ||
        j < 0 || j >= static_cast<int>(points.size())) {
      throw std::runtime_error("Edge indices are out of bounds.");
    }
    const Segment2 seg(points[i], points[j]);
    CGAL::insert(arrangement, seg, point_location);
  }

  // Add convex hull edges
  std::vector<Point> hull;
  hull.reserve(points.size());
  CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(hull));
  for (size_t k = 0; k < hull.size(); ++k) {
    const Point &p1 = hull[k];
    const Point &p2 = hull[(k + 1) % hull.size()];
    const Segment2 hull_edge(p1, p2);
    CGAL::insert(arrangement, hull_edge, point_location);
  }
}

/**
 * Extract triangular faces from the arrangement.
 * Expects that all bounded faces are triangles and will throw if not.
 */
static std::vector<std::tuple<int, int, int>>
extract_triangular_faces(
    const Arrangement_2 &arrangement,
    const std::map<Point, int, LessPointXY> &idx_of) {

  std::vector<std::tuple<int, int, int>> triangles;
  triangles.reserve(arrangement.number_of_faces());

  for (auto fit = arrangement.faces_begin(); fit != arrangement.faces_end(); ++fit) {
    if (fit->is_unbounded()) continue;

    // Walk the boundary and collect vertex indices
    std::array<int, 3> idxs;
    int deg = 0;

    Halfedge_const_handle e = fit->outer_ccb();
    Halfedge_const_handle start = e;

    do {
      if (deg > 3) {throw std::runtime_error("Bound face is not triangular.");}; // Early out: not a triangle

      const Point &pv = e->source()->point();
      auto it = idx_of.find(pv);
      if (it == idx_of.end()) {
        // Vertex not in original points (likely intersection) - skip face
        throw std::runtime_error("Face vertex not found in original points list.");
      }
      if (deg < 3) idxs[deg] = it->second;
      ++deg;

      e = e->next();
    } while (e != start);

    if (deg == 3) {
      // Canonicalize order and add triangle
      std::sort(idxs.begin(), idxs.end());
      triangles.emplace_back(idxs[0], idxs[1], idxs[2]);
    }
  }

  return triangles;
}

// ============================================================================
// compute_triangles - Main function
// ============================================================================

/**
 * This function computes all triangles formed by the given set of points and
 * edges. It returns a list of triangles, where each triangle is represented by
 * a tuple of three point indices. Edges that appear only once will be on the
 * convex hull. Otherwise, all edges should appear exactly twice. The indices
 * will be sorted in each triangle, and the list of triangles will also be
 * sorted.
 * Expects that all bounded faces are triangles and will throw if not.
 */
std::vector<std::tuple<int, int, int>>
compute_triangles(const std::vector<Point> &points,
                  const std::vector<std::tuple<int, int>> &edges) {
  // Step 1: Build point-to-index mapping
  std::map<Point, int, LessPointXY> idx_of;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    idx_of.emplace(points[i], i);
  }

  // Step 2: Build arrangement with edges and convex hull
  Arrangement_2 arrangement;
  PointLocation point_location(arrangement);
  build_arrangement_for_triangles(points, edges, arrangement, point_location);

  // Step 3: Extract triangular faces
  std::vector<std::tuple<int, int, int>> triangles =
      extract_triangular_faces(arrangement, idx_of);

  // Step 4: Sort
  std::sort(triangles.begin(), triangles.end());
  const auto num_triangles = triangles.size();
  triangles.erase(std::unique(triangles.begin(), triangles.end()),
                  triangles.end());
  if (triangles.size() != num_triangles) {
    throw std::runtime_error("Duplicate triangles found after extraction. This should not happen.");
  }

  return triangles;
}

} // namespace cgshop2026
