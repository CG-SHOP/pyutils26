#include "triangulation_validation.h"
#include <CGAL/convex_hull_2.h>
#include <fmt/core.h>

namespace cgshop2026 {

std::optional<std::map<Point, int, LessPointXY>>
build_point_index_map(const std::vector<Point> &points, bool verbose) {
  std::map<Point, int, LessPointXY> idx_of;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    idx_of.emplace(points[i], i);
    if (idx_of.size() != static_cast<size_t>(i + 1)) {
      if (verbose)
        fmt::print("ERROR: Duplicate point found at index {}: {}\n", i,
                   point_to_string(points[i]));
      return std::nullopt;
    }
  }
  return idx_of;
}

bool insert_edges_into_arrangement(
    const std::vector<Point> &points,
    const std::vector<std::tuple<int, int>> &edges,
    Arrangement_2 &arrangement,
    PointLocation &point_location,
    bool verbose) {

  if (verbose)
    fmt::print("Inserting {} edges into arrangement.\n", edges.size());

  for (size_t edge_idx = 0; edge_idx < edges.size(); ++edge_idx) {
    const auto &edge = edges[edge_idx];
    int i = std::get<0>(edge);
    int j = std::get<1>(edge);

    if (i < 0 || i >= static_cast<int>(points.size()) ||
        j < 0 || j >= static_cast<int>(points.size())) {
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

    // Ensure only a single new segment is added per edge
    if (arrangement.number_of_edges() != edge_idx + 1) {
      if (verbose)
        fmt::print("ERROR: Inserting edge {} created multiple segments. "
                   "Arrangement edges: {}\n",
                   edge_idx, arrangement.number_of_edges());
      return false;
    }
  }
  return true;
}

void add_convex_hull_to_arrangement(
    const std::vector<Point> &points,
    Arrangement_2 &arrangement,
    PointLocation &point_location,
    bool verbose) {

  std::vector<Point> hull;
  CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(hull));

  if (verbose)
    fmt::print(
        "Convex hull has {} vertices. Adding hull edges if not present.\n",
        hull.size());

  for (size_t k = 0; k < hull.size(); ++k) {
    const Point &p1 = hull[k];
    const Point &p2 = hull[(k + 1) % hull.size()];
    Segment2 hull_edge(p1, p2);

    if (verbose)
      fmt::print("  Hull edge {}: {} -> {}\n", k, point_to_string(p1),
                 point_to_string(p2));

    CGAL::insert(arrangement, hull_edge, point_location);
  }
}

bool validate_vertex_count(
    const Arrangement_2 &arrangement,
    size_t expected_count,
    const std::vector<Point> &points,
    bool verbose) {

  if (verbose)
    fmt::print("Initial vertex count: {}, Arrangement vertex count: {}\n",
               expected_count, arrangement.number_of_vertices());

  if (arrangement.number_of_vertices() > expected_count) {
    if (verbose) {
      fmt::print(
          "ERROR: New intersection points were created. Expected {}, got {}\n",
          expected_count, arrangement.number_of_vertices());
      fmt::print("Arrangement vertices:\n");
      size_t vertex_idx = 0;
      for (auto v_it = arrangement.vertices_begin();
           v_it != arrangement.vertices_end(); ++v_it, ++vertex_idx) {
        fmt::print("  Vertex {}: {}\n", vertex_idx,
                   point_to_string(v_it->point()));
      }
    }
    return false;
  }

  if (arrangement.number_of_vertices() < expected_count) {
    if (verbose) {
      fmt::print(
          "ERROR: Points are missing in arrangement. Expected {}, got {}\n",
          expected_count, arrangement.number_of_vertices());
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

  return true;
}

bool validate_all_faces_triangular(
    const Arrangement_2 &arrangement,
    const std::map<Point, int, LessPointXY> &idx_of,
    std::unordered_set<std::tuple<int, int>, TupleHash> &edges_in_arrangement,
    bool verbose) {

  if (verbose)
    fmt::print("Checking for non-triangular faces.\n");

  size_t face_count = 0;
  size_t triangular_faces = 0;

  for (auto fit = arrangement.faces_begin(); fit != arrangement.faces_end();
       ++fit, ++face_count) {
    if (fit->is_unbounded()) {
      if (verbose)
        fmt::print("  Face {}: Unbounded (skipped)\n", face_count);
      continue;
    }

    // Count edges and collect vertices
    int edge_count = 0;
    Halfedge_const_handle e = fit->outer_ccb();
    std::vector<Point> face_vertices;

    do {
      edge_count++;
      face_vertices.push_back(e->source()->point());
      e = e->next();
    } while (e != fit->outer_ccb());

    // Check if face is a triangle
    if (edge_count != 3) {
      if (verbose) {
        fmt::print("ERROR: Face {} has {} edges (expected 3)\n",
                   face_count, edge_count);
        fmt::print("    Vertices: ");
        for (size_t v = 0; v < face_vertices.size(); ++v) {
          fmt::print("{}{}", point_to_string(face_vertices[v]),
                     (v < face_vertices.size() - 1) ? ", " : "\n");
        }
      }
      return false;
    }

    triangular_faces++;
    if (verbose)
      fmt::print("  Face {}: Triangle with vertices: {}, {}, {}\n",
                 face_count, point_to_string(face_vertices[0]),
                 point_to_string(face_vertices[1]),
                 point_to_string(face_vertices[2]));

    // Collect vertex indices and edges
    std::vector<int> vertex_indices;
    e = fit->outer_ccb();
    do {
      const Point &p = e->source()->point();
      auto it = idx_of.find(p);
      if (it == idx_of.end()) {
        if (verbose)
          fmt::print("ERROR: Face vertex {} not found in original points list.\n",
                     point_to_string(p));
        return false;
      }
      vertex_indices.push_back(it->second);
      e = e->next();
    } while (e != fit->outer_ccb());

    // Add the three edges of this triangle
    edges_in_arrangement.emplace(vertex_indices[0], vertex_indices[1]);
    edges_in_arrangement.emplace(vertex_indices[1], vertex_indices[2]);
    edges_in_arrangement.emplace(vertex_indices[2], vertex_indices[0]);
  }

  if (verbose)
    fmt::print("  Total bounded faces: {}, Triangular: {}\n",
               face_count - 1, triangular_faces);

  return true;
}

bool validate_input_edges_present(
    const std::vector<std::tuple<int, int>> &edges,
    const std::unordered_set<std::tuple<int, int>, TupleHash> &edges_in_arrangement,
    bool verbose) {

  for (const auto &edge : edges) {
    // Check both orientations
    if (edges_in_arrangement.count(edge) == 0 &&
        edges_in_arrangement.count(std::make_tuple(std::get<1>(edge),
                                                     std::get<0>(edge))) == 0) {
      if (verbose)
        fmt::print("ERROR: Edge ({}, {}) from input not found in arrangement.\n",
                   std::get<0>(edge), std::get<1>(edge));
      return false;
    }
  }
  return true;
}

} // namespace cgshop2026
