#include "geometry_operations.h"
#include <CGAL/convex_hull_2.h>
#include <algorithm>
#include <array>
#include <iostream>
#include <map>
#include <set>
#include <unordered_set>

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


/**
 * This function checks if the given set of edges forms a triangulation of the
 * provided points. It uses the CGAL arrangement data structure to insert the
 * edges and verify the triangulation properties.
 */
bool is_triangulation(const std::vector<Point> &points,
                      const std::vector<std::tuple<int, int>> &edges,
                      bool verbose) {
  std::map<Point, int, LessPointXY> idx_of;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    idx_of.emplace(points[i], i);
    // expect the idx_of to be of size i+1 after this insertion
    // otherwise, a duplicate point was inserted
    if (idx_of.size() != i + 1) {
      if (verbose)
        fmt::print("ERROR: Duplicate point found at index {}: {}\n", i,
                   point_to_string(points[i]));
      return false; // Duplicate point found
    }
  }
  // Create an arrangement to hold the edges
  Arrangement_2 arrangement;
  PointLocation point_location(arrangement);

  // Store initial number of vertices to check for new intersections
  size_t initial_vertex_count = points.size();

  // Insert the edges into the arrangement
  if (verbose)
    fmt::print("Inserting {} edges into arrangement.\n", edges.size());
  for (size_t edge_idx = 0; edge_idx < edges.size(); ++edge_idx) {
    const auto &edge = edges[edge_idx];
    int i = std::get<0>(edge);
    int j = std::get<1>(edge);
    if (i < 0 || i >= points.size() || j < 0 || j >= points.size()) {
      if (verbose)
        fmt::print(
            "ERROR: Edge {} has invalid indices ({}, {}). Point count: {}\n",
            edge_idx, i, j, points.size());
      throw std::runtime_error("Edge indices are out of bounds.");
    }
    Segment2 segment(points[i], points[j]);
    if (verbose)
      fmt::print("  Edge {}: {} -> {} (points {} to {})\n", edge_idx,
                 point_to_string(points[i]), point_to_string(points[j]), i, j);
    CGAL::insert(arrangement, segment, point_location);
    // ensure that only a single new segment is added per edge
    if (arrangement.number_of_edges() != edge_idx + 1) {
      if (verbose)
        fmt::print("ERROR: Inserting edge {} created multiple segments. "
                   "Arrangement edges: {}\n",
                   edge_idx, arrangement.number_of_edges());
      return false; // Edge insertion created multiple segments
    }
  }

  // Automatically add convex hull edges if not present
  std::vector<Point> hull;
  CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(hull));
  if (verbose)
    fmt::print(
        "Convex hull has {} vertices. Adding hull edges if not present.\n",
        hull.size());
  for (size_t k = 0; k < hull.size(); ++k) {
    Point p1 = hull[k];
    Point p2 = hull[(k + 1) % hull.size()];
    Segment2 hull_edge(p1, p2);
    if (verbose)
      fmt::print("  Hull edge {}: {} -> {}\n", k, point_to_string(p1),
                 point_to_string(p2));
    CGAL::insert(arrangement, hull_edge, point_location);
  }

  if (verbose)
    fmt::print("Checking triangulation properties.\n");
  if (verbose)
    fmt::print("Initial vertex count: {}, Arrangement vertex count: {}\n",
               initial_vertex_count, arrangement.number_of_vertices());

  // Check that no new vertices were created by intersections
  std::unordered_set<std::tuple<int, int>, TupleHash> edges_in_arrangement;
  if (arrangement.number_of_vertices() > initial_vertex_count) {
    if (verbose)
      fmt::print(
          "ERROR: New intersection points were created. Expected {}, got {}\n",
          initial_vertex_count, arrangement.number_of_vertices());

    // List all vertices in the arrangement to help debug
    if (verbose)
      fmt::print("Arrangement vertices:\n");
    size_t vertex_idx = 0;
    for (auto v_it = arrangement.vertices_begin();
         v_it != arrangement.vertices_end(); ++v_it, ++vertex_idx) {
      if (verbose)
        fmt::print("  Vertex {}: {}\n", vertex_idx,
                   point_to_string(v_it->point()));
    }
    return false;
  }
  if (arrangement.number_of_vertices() < initial_vertex_count) {
    if (verbose)
      fmt::print(
          "ERROR: Points are missing in arrangement. Expected {}, got {}\n",
          initial_vertex_count, arrangement.number_of_vertices());

    // List all vertices in the arrangement to help debug
    if (verbose) {
      fmt::print("Arrangement vertices:\n");
      size_t vertex_idx = 0;
      for (auto v_it = arrangement.vertices_begin();
         v_it != arrangement.vertices_end(); ++v_it, ++vertex_idx) {
      fmt::print("  Vertex {}: {}\n", vertex_idx,
             point_to_string(v_it->point()));
      }
      fmt::print("Original vertices:\n");
      for (size_t i = 0; i < points.size(); ++i) {
      fmt::print("  Point {}: {}\n", i, point_to_string(points[i]));
      }
    }
    return false;
  }

  if (verbose)
    fmt::print("Checking for non-triangular faces.\n");
  size_t face_count = 0;
  size_t triangular_faces = 0;
  size_t non_triangular_faces = 0;

  // Check if all faces in the arrangement are triangles
  for (auto it = arrangement.faces_begin(); it != arrangement.faces_end();
       ++it, ++face_count) {
    if (it->is_unbounded()) {
      if (verbose)
        fmt::print("  Face {}: Unbounded (skipped)\n", face_count);
      continue;
    }

    // Count the number of edges in the face
    int edge_count = 0;
    Halfedge_const_handle e = it->outer_ccb();
    std::vector<Point> face_vertices;
    do {
      edge_count++;
      face_vertices.push_back(e->source()->point());
      e = e->next();
    } while (e != it->outer_ccb());

    if (edge_count == 3) {
      triangular_faces++;
      if (verbose)
        fmt::print("  Face {}: Triangle with vertices: {}, {}, {}\n",
                   face_count, point_to_string(face_vertices[0]),
                   point_to_string(face_vertices[1]),
                   point_to_string(face_vertices[2]));
    } else {
      non_triangular_faces++;
      if (verbose)
        fmt::print("  Face {}: Non-triangular with {} edges\n", face_count,
                   edge_count);
      if (verbose)
        fmt::print("    Vertices: ");
      for (size_t v = 0; v < face_vertices.size(); ++v) {
        if (verbose)
          fmt::print("{}{}", point_to_string(face_vertices[v]),
                     (v < face_vertices.size() - 1) ? ", " : "\n");
      }
    }

    // If any face has more than 3 edges, it's not a triangulation
    if (edge_count != 3) {
      if (verbose)
        fmt::print("ERROR: Face with {} edges found (expected 3)\n",
                   edge_count);
      return false;
    }

    // Collect the vertices of the face
    std::vector<int> vertex_indices;
    do {
      Point p = e->source()->point();
      auto it = idx_of.find(p);
      if (it != idx_of.end()) {
        vertex_indices.push_back(it->second);
      } else {
        if (verbose)
          fmt::print("ERROR: Face vertex {} not found in original points list.\n",
                     point_to_string(p));
        return false;
      }
      e = e->next();
    } while (e != it->outer_ccb());

    edges_in_arrangement.emplace(vertex_indices[0], vertex_indices[1]);
    edges_in_arrangement.emplace(vertex_indices[1], vertex_indices[2]);
    edges_in_arrangement.emplace(vertex_indices[2], vertex_indices[0]);
  }

  // check that all edge also appear in the arrangement
  for(const auto &edge : edges) {
    if (edges_in_arrangement.count(edge) == 0 &&
        edges_in_arrangement.count(std::make_tuple(std::get<1>(edge), std::get<0>(edge))) == 0) {
      if (verbose)
        fmt::print("ERROR: Edge ({}, {}) from faces not found in arrangement.\n",
                   std::get<0>(edge), std::get<1>(edge));
      return false;
    }
  }

  if (verbose)
    fmt::print("Triangulation check complete:\n");
  if (verbose)
    fmt::print("  Total faces: {}\n", face_count);
  if (verbose)
    fmt::print("  Triangular faces: {}\n", triangular_faces);
  if (verbose)
    fmt::print("  Non-triangular faces: {}\n", non_triangular_faces);
  if (verbose)
    fmt::print("  Result: Valid triangulation\n");

  return true; // All faces are triangles
}

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
                  const std::vector<std::tuple<int, int>>& edges) {
  std::cout<<"Computing triangles from " << points.size() << " points and "
            << edges.size() << " edges." << std::endl;
  // ---- 1) Pre-index all points: Point -> index (O(n log n))
  std::map<Point, int, LessPointXY> idx_of;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    idx_of.emplace(points[i], i);
  }

  // ---- 2) Build arrangement
  Arrangement_2 arrangement;
  PointLocation point_location(arrangement);

  // Insert input edges
  for (const auto& edge : edges) {
    const int i = std::get<0>(edge);
    const int j = std::get<1>(edge);
    if (i < 0 || i >= static_cast<int>(points.size())
        || j < 0 || j >= static_cast<int>(points.size())) {
      throw std::runtime_error("Edge indices are out of bounds.");
    }
    const Segment2 seg(points[i], points[j]);
    CGAL::insert(arrangement, seg, point_location);
  }

  // Add convex hull edges (if not present)
  std::vector<Point> hull;
  hull.reserve(points.size());
  CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(hull));
  for (size_t k = 0; k < hull.size(); ++k) {
    const Point& p1 = hull[k];
    const Point& p2 = hull[(k + 1) % hull.size()];
    const Segment2 hull_edge(p1, p2);
    CGAL::insert(arrangement, hull_edge, point_location);
  }

  // ---- 3) Extract triangular faces
  std::vector<std::tuple<int, int, int>> triangles;
  triangles.reserve(arrangement.number_of_faces()); // rough upper bound

  for (auto fit = arrangement.faces_begin(); fit != arrangement.faces_end(); ++fit) {
    if (fit->is_unbounded()) continue;

    // Walk the boundary once; count degree quickly
    std::array<int, 3> idxs;
    int deg = 0;

    Halfedge_const_handle e = fit->outer_ccb();
    Halfedge_const_handle start = e;

    do {
      if (deg > 3) break; // early out: not a triangle

      const Point& pv = e->source()->point();
      auto it = idx_of.find(pv);
      if (it == idx_of.end()) {
        // This vertex wasn't one of the original points (likely an intersection).
        // Skip this face to mirror your original behavior but without throwing.
        deg = 999; // mark as invalid
        break;
      }
      if (deg < 3) idxs[deg] = it->second;
      ++deg;

      e = e->next();
    } while (e != start);

    if (deg == 3) {
      // canonicalize order
      std::sort(idxs.begin(), idxs.end());
      triangles.emplace_back(idxs[0], idxs[1], idxs[2]);
    }
  }

  // (Optional) deduplicate triangles if your arrangement can produce duplicates
  std::sort(triangles.begin(), triangles.end());
  triangles.erase(std::unique(triangles.begin(), triangles.end()), triangles.end());

  return triangles;
}

} // namespace cgshop2026
