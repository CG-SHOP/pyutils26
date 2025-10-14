// pybind11
#include <pybind11/operators.h> // To define operator overloading
#include <pybind11/pybind11.h>  // Basic pybind11 functionality
#include <pybind11/stl.h>       // Automatic conversion of vectors

// cgal
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_integer.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Rotational_sweep_visibility_2.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/create_straight_skeleton_2.h>

// fmt
#include <exception>
#include <fmt/core.h>
#include <unordered_map>

// for duplicate check
#include <map>

namespace cgshop2026 {
// Define CGAL types for easy readability and maintenance
using Kernel = CGAL::Epeck; // Exact Predicates Exact Constructions Kernel
using Point = CGAL::Point_2<Kernel>;
using Polygon2WithHoles = CGAL::Polygon_with_holes_2<Kernel>;
using Polygon2 = CGAL::Polygon_2<Kernel>;
using Segment2 = Kernel::Segment_2;
using Traits_2 = CGAL::Arr_segment_traits_2<Kernel>;
using Arrangement_2 = CGAL::Arrangement_2<Traits_2>;
using Halfedge_const_handle = Arrangement_2::Halfedge_const_handle;
using Face_handle = Arrangement_2::Face_handle;
using PointLocation = CGAL::Arr_naive_point_location<Arrangement_2>;
using Rational = CGAL::Gmpq;

std::string point_to_string(const Point &p) {
  return fmt::format("({}, {})", CGAL::to_double(p.x()),
                     CGAL::to_double(p.y()));
}

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
                      bool verbose = false) {
  // Create an arrangement to hold the edges
  Arrangement_2 arrangement;
  PointLocation point_location(arrangement);
  
  // Check that each point is unique
  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      if (points[i] == points[j]) {
        if (verbose)
          fmt::print("ERROR: Duplicate points found at indices {} and {}: {}\n",
                     i, j, point_to_string(points[i]));
        return false; // Duplicate points found
      }
    }
  }

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
  std::vector<std::tuple<int, int>> edges_in_arrangement;
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
      auto it = std::find(points.begin(), points.end(), p);
      if (it != points.end()) {
        vertex_indices.push_back(std::distance(points.begin(), it));
      } else {
        if (verbose)
          fmt::print("ERROR: Face vertex {} not found in original points list.\n",
                     point_to_string(p));
        return false;
      }
      e = e->next();
    } while (e != it->outer_ccb());

    edges_in_arrangement.emplace_back(vertex_indices[0], vertex_indices[1]);
    edges_in_arrangement.emplace_back(vertex_indices[1], vertex_indices[2]);
    edges_in_arrangement.emplace_back(vertex_indices[2], vertex_indices[0]);
  }

  // check that all edge also appear in the arrangement
  for(const auto &edge : edges) {
    if (std::find(edges_in_arrangement.begin(), edges_in_arrangement.end(),
                  edge) == edges_in_arrangement.end() &&
        std::find(edges_in_arrangement.begin(), edges_in_arrangement.end(),
                  std::make_tuple(std::get<1>(edge), std::get<0>(edge))) ==
            edges_in_arrangement.end()) {
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
compute_triangles(const std::vector<Point> &points,
                  const std::vector<std::tuple<int, int>> &edges) {
  // Create an arrangement to hold the edges
  Arrangement_2 arrangement;
  PointLocation point_location(arrangement);

  // Insert the edges into the arrangement
  for (const auto &edge : edges) {
    int i = std::get<0>(edge);
    int j = std::get<1>(edge);
    if (i < 0 || i >= points.size() || j < 0 || j >= points.size()) {
      throw std::runtime_error("Edge indices are out of bounds.");
    }
    Segment2 segment(points[i], points[j]);
    CGAL::insert(arrangement, segment, point_location);
  }

  // Automatically add convex hull edges if not present
  std::vector<Point> hull;
  CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(hull));
  for (size_t k = 0; k < hull.size(); ++k) {
    Point p1 = hull[k];
    Point p2 = hull[(k + 1) % hull.size()];
    Segment2 hull_edge(p1, p2);
    CGAL::insert(arrangement, hull_edge, point_location);
  }

  // Extract triangles from the arrangement
  std::vector<std::tuple<int, int, int>> triangles;
  for (auto it = arrangement.faces_begin(); it != arrangement.faces_end();
       ++it) {
    if (it->is_unbounded())
      continue;

    // Collect the vertices of the face
    std::vector<int> vertex_indices;
    Halfedge_const_handle e = it->outer_ccb();
    do {
      Point p = e->source()->point();
      auto it = std::find(points.begin(), points.end(), p);
      if (it != points.end()) {
        vertex_indices.push_back(std::distance(points.begin(), it));
      } else {
        throw std::runtime_error(
            "Face vertex not found in original points list.");
      }
      e = e->next();
    } while (e != it->outer_ccb());

    // Only consider triangular faces
    if (vertex_indices.size() == 3) {
      std::sort(vertex_indices.begin(), vertex_indices.end());
      triangles.emplace_back(vertex_indices[0], vertex_indices[1],
                             vertex_indices[2]);
    }
  }

  return triangles;
}

// Compute convex hull and return the indices of the points on the hull
std::vector<int64_t> compute_convex_hull(const std::vector<Point> &points) {
  // Compute convex hull
  std::vector<Point> hull;
  CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(hull));

  // Find the indices of the points in the original vector
  std::vector<int64_t> result;
  for (const auto &p : hull) {
    auto it = std::find(points.begin(), points.end(), p);
    if (it != points.end()) {
      result.push_back(std::distance(points.begin(), it));
    } else {
      throw std::runtime_error(
          "Point on hull not found in original points list.");
    }
  }
  return result;
}

// Helper class to manage geometry and verify solutions using CGAL's
// arrangements
class VerificationGeometryHelper {
public:
  VerificationGeometryHelper()
      : arrangement_(), point_location_(arrangement_) {}

  // Add a point to the arrangement and return its index
  int add_point(const Point &p) {
    CGAL::insert_point(arrangement_, p, point_location_);
    points_.push_back(p);
    return static_cast<int>(points_.size() - 1);
  }

  // Add a segment between two points in the arrangement
  void add_segment(const int i, const int j) {
    const Point &p1 = points_[i];
    const Point &p2 = points_[j];
    Segment2 s(p1, p2);
    CGAL::insert(arrangement_, s, point_location_);
  }

  // Get the number of points in the arrangement
  int get_num_points() const {
    return static_cast<int>(arrangement_.number_of_vertices());
  }

  // Search for any bounded face that is not triangular
  std::optional<Point> search_for_non_triangular_faces() const {
    for (auto it = arrangement_.faces_begin(); it != arrangement_.faces_end();
         ++it) {
      if (it->is_unbounded())
        continue;

      // Count the number of edges in the outer boundary
      int num_edges = 0;
      Halfedge_const_handle e = it->outer_ccb();
      do {
        num_edges++;
        e = e->next();
      } while (e != it->outer_ccb());

      // If the face is not a triangle, return a point inside the face
      if (num_edges != 3) {
        return e->source()->point();
      }
    }
    return std::nullopt;
  }

  // Search for faces with holes and return a point in such a face
  std::optional<Point> search_for_faces_with_holes() const {
    for (auto it = arrangement_.faces_begin(); it != arrangement_.faces_end();
         ++it) {
      if (!it->is_unbounded() && it->number_of_holes() > 0) {
        return it->outer_ccb()->source()->point();
      }
    }
    return std::nullopt;
  }

  // Count obtuse triangles in bounded faces
  int count_obtuse_triangles() const {
    int count = 0;
    for (auto it = arrangement_.faces_begin(); it != arrangement_.faces_end();
         ++it) {
      if (it->is_unbounded())
        continue;

      // Get the three vertices of the face
      auto e = it->outer_ccb();
      Point p1 = e->source()->point();
      Point p2 = e->target()->point();
      e = e->next();
      Point p3 = e->target()->point();

      // Compute all three dot products to check for obtuse angles
      std::array<Point, 3> points = {p1, p2, p3};
      for (int i = 0; i < 3; i++) {
        auto v1 = points[(i + 1) % 3] - points[i];
        auto v2 = points[(i + 2) % 3] - points[i];
        if (CGAL::scalar_product(v1, v2) < 0) {
          count++;
          break;
        }
      }
    }
    return count;
  }

  // Get points that are isolated (not connected by any edge)
  std::vector<Point> search_for_isolated_points() const {
    std::vector<Point> isolated_points;
    for (auto it = arrangement_.vertices_begin();
         it != arrangement_.vertices_end(); ++it) {
      if (it->is_isolated()) {
        isolated_points.push_back(it->point());
      }
    }
    return isolated_points;
  }

  // Search for segments that have the same face on both sides
  std::optional<Segment2> search_for_bad_edges() const {
    for (auto it = arrangement_.edges_begin(); it != arrangement_.edges_end();
         ++it) {
      if (it->face() == it->twin()->face()) {
        return it->curve();
      }
    }
    return std::nullopt;
  }

protected:
  std::vector<Point> points_;    // Store the points in the arrangement
  Arrangement_2 arrangement_;    // The CGAL arrangement of segments and points
  PointLocation point_location_; // A point location utility for fast querying
};

// Define CGAL types for constrained triangulation
using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<
    Kernel, Tds, CGAL::No_constraint_intersection_tag>;

class ConstrainedTriangulation {
public:
  ConstrainedTriangulation() : cdt_() {}

  // Add a point to the triangulation and return its index
  int add_point(const Point &p) {
    auto index = points_.size();
    auto v = cdt_.insert(p);
    points_.push_back(v);
    point_to_index_[v] = index;
    return static_cast<int>(index);
  }

  // Add a boundary made of points and verify if it is valid
  void add_boundary(const std::vector<int> &boundary) {
    std::vector<Point> vertices;
    vertices.reserve(boundary.size());
    for (auto i : boundary) {
      vertices.push_back(points_[i]->point());
    }
    cdt_.insert_constraint(vertices.begin(), vertices.end(), true);
    boundary_ = Polygon2(vertices.begin(), vertices.end());

    // Ensure the boundary is a simple, counter-clockwise polygon
    if (!boundary_->is_simple()) {
      throw std::runtime_error("Boundary must be a simple polygon.");
    }
    if (boundary_->is_clockwise_oriented()) {
      throw std::runtime_error("Boundary must be counter-clockwise oriented.");
    }
  }

  // Add a segment between two points in the triangulation
  void add_segment(const int i, const int j) {
    auto p1 = points_[i];
    auto p2 = points_[j];
    cdt_.insert_constraint(p1, p2);
  }

  // Get the edges of the triangulation, ignoring those outside the boundary
  std::vector<std::tuple<int, int>> get_triangulation_edges() const {
    std::vector<std::tuple<int, int>> edges;
    for (auto e = cdt_.finite_edges_begin(); e != cdt_.finite_edges_end();
         ++e) {
      auto v1 = e->first->vertex((e->second + 1) % 3);
      auto v2 = e->first->vertex((e->second + 2) % 3);
      auto middle = CGAL::midpoint(v1->point(), v2->point());

      // Ignore edges outside the boundary
      if (boundary_.has_value() && boundary_->has_on_unbounded_side(middle)) {
        continue;
      }
      auto i = point_to_index_.at(v1);
      auto j = point_to_index_.at(v2);
      edges.emplace_back(i, j);
    }
    return edges;
  }

protected:
  std::optional<Polygon2> boundary_; // Store the boundary polygon
  std::vector<CDT::Vertex_handle>
      points_; // Store the points in the triangulation
  std::unordered_map<CDT::Vertex_handle, int>
      point_to_index_; // Map points to their indices
  CDT cdt_;            // The constrained Delaunay triangulation
};

// Function to convert Kernel::FT to rational string
std::string to_rational_string(const Kernel::FT &x) {
  auto exact_value = CGAL::exact(x);
  std::string exact_str = exact_value.str();
  return exact_str;
}

// Exact conversion of a long to CGAL's Field Type (FT)
auto to_exact(std::int64_t x) {
  double lo32 = x & 0xffff'ffff;
  double hi32 = static_cast<double>(x >> 32) * 4294967296.0;
  return Kernel::FT(hi32) + Kernel::FT(lo32);
}

std::optional<Point> intersection_point(const Segment2 &s1,
                                        const Segment2 &s2) {
  auto result = CGAL::intersection(s1, s2);
  if (result) {
    if (const Point *p = std::get_if<Point>(&*result)) {
      return *p;
    }
  }
  return std::nullopt;
}

template <typename ER = CGAL::Exact_rational, typename EI = CGAL::Exact_integer>
static CGAL::Exact_rational integer_str_to_exact(const std::string &str) {
  if constexpr (std::is_constructible_v<ER, const std::string &>) {
    return CGAL::Exact_rational(str);
  } else if (std::is_constructible_v<ER, const char *>) {
    return CGAL::Exact_rational(str.c_str());
  } else if (std::is_constructible_v<EI, const std::string &> &&
             std::is_constructible_v<ER, EI>) {
    return CGAL::Exact_rational(CGAL::Exact_integer(str));
  } else if (std::is_constructible_v<EI, const char *> &&
             std::is_constructible_v<ER, EI>) {
    return CGAL::Exact_rational(CGAL::Exact_integer(str.c_str()));
  } else {
    // fallback to I/O operators
    std::istringstream input(str);
    CGAL::Exact_rational exact(0);
    input >> exact;
    return exact;
  }
}

static void remove_whitespace(std::string &number) {
  // restricted whitespace detection;
  // cannot use std::isspace (negative chars cause UB)
  auto is_ws = [](char c) -> bool {
    return c == ' ' || c == '\t' || c == '\n';
  };
  number.erase(std::remove_if(number.begin(), number.end(), is_ws),
               number.end());
}

static void check_allowed(const std::string &number) {
  auto is_allowed = [](char c) -> bool {
    if (c < 0)
      return false;
    return std::isdigit(c) || c == '/' || c == '-';
  };
  if (!std::all_of(number.begin(), number.end(), is_allowed)) {
    throw std::runtime_error("Invalid character in number string; only "
                             "integers and string ratios are allowed.");
  }
}

static void check_sign(const std::string &number) {
  if (number.empty())
    return;
  if (!std::all_of(number.begin() + 1, number.end(),
                   [](char c) { return std::isdigit(c); })) {
    throw std::runtime_error(
        "Negative sign character '-' in invalid position in number string.");
  }
}

static Kernel::FT checked_int_str_to_exact(std::string number) {
  check_sign(number);
  constexpr size_t max_len = 16;
  if (number.length() <= max_len) {
    if (number.empty())
      return 0;
    return to_exact(std::int64_t(std::stoll(number)));
  }
  return Kernel::FT(integer_str_to_exact(number));
}

// Convert string to exact number (Kernel::FT), handling rational numbers
Kernel::FT str_to_exact(std::string number) {
  // remove whitespaces, leading plus signs, and leading zeros
  remove_whitespace(number);
  number.erase(0, number.find_first_not_of('+'));
  number.erase(0, number.find_first_not_of('0'));
  if (number.empty()) {
    return 0;
  }
  check_allowed(number);
  std::size_t slash_pos = number.find('/');
  if (slash_pos != std::string::npos) {
    if (number.find('/', slash_pos + 1) != std::string::npos) {
      throw std::runtime_error("More than one / in number string!");
    }
    // rational
    auto numerator = checked_int_str_to_exact(number.substr(0, slash_pos));
    auto denominator = checked_int_str_to_exact(number.substr(slash_pos + 1));
    if (denominator == 0) {
      throw std::runtime_error("Divide by 0 in number string!");
    }
    return numerator / denominator;
  }
  // (possibly signed) integer
  return checked_int_str_to_exact(std::move(number));
}

std::optional<std::pair<int, int>>
points_contain_duplicates(const std::vector<Point> &points) {
  std::map<Point, std::size_t> unique_points;
  std::size_t index = 0;
  for (const auto &p : points) {
    auto result = unique_points.try_emplace(p, index);
    if (!result.second) {
      return std::pair<int, int>{int(result.first->second), int(index)};
    }
    ++index;
  }
  return std::nullopt;
}
}; // namespace cgshop2026

// Pybind11 module definitions
PYBIND11_MODULE(_bindings, m) {
  namespace py = pybind11;
  using namespace cgshop2026;
  m.doc() = "Example of PyBind11 and CGAL."; // Optional module docstring

  // Exact numbers
  py::class_<Kernel::FT>(m, "FieldNumber",
                         "A container for exact numbers in CGAL.")
      .def(py::init<long>())
      .def(py::init<double>())
      .def(py::init(&str_to_exact))
      .def(py::self / Kernel::FT())
      .def(py::self + Kernel::FT())
      .def(py::self - Kernel::FT())
      .def(py::self * Kernel::FT())
      .def(py::self == Kernel::FT())
      .def(py::self < Kernel::FT())
      .def(py::self > Kernel::FT())
      .def(py::self <= Kernel::FT())
      .def(py::self >= Kernel::FT())
      .def("__float__",
           [](const Kernel::FT &ft) { return CGAL::to_double(ft); })
      .def("__str__",
           [](const Kernel::FT &x) {
             return std::to_string(CGAL::to_double(x));
           })
      .def("exact", &to_rational_string);

  // Points
  py::class_<Point>(m, "Point", "A 2-dimensional point.")
      .def(py::init<long, long>())
      .def(py::init<double, double>())
      .def(py::init<Kernel::FT, Kernel::FT>())
      .def("__add__",
           [](const Point &p1, const Point &p2) {
             // Addition is not defined in CGAL for points (?!)
             return Point(p1.x() + p2.x(), p1.y() + p2.y());
           })
      .def("__sub__",
           [](const Point &p1, const Point &p2) {
             return Point(p1.x() - p2.x(), p1.y() - p2.y());
           })
      .def(py::self == Point())
      .def(py::self != Point())
      .def("scale",
           [](const Point &p, const Kernel::FT &s) {
             return Point(p.x() * s, p.y() * s);
           })
      .def("x", [](const Point &p) { return p.x(); })
      .def("y", [](const Point &p) { return p.y(); })
      .def("__len__", [](const Point &self) { return 2; })
      .def("__getitem__",
           [](const Point &self, int i) {
             if (i == 0) {
               return self.x();
             } else if (i == 1) {
               return self.y();
             }
             throw std::out_of_range("Only 0=x and 1=y.");
           })
      .def(py::self == Point())
      .def("__str__", &point_to_string);

  // Segments
  py::class_<Segment2>(m, "Segment", "A 2-dimensional segment.")
      .def(py::init<Point, Point>())
      .def("source", &Segment2::source)
      .def("target", &Segment2::target)
      .def("squared_length", &Segment2::squared_length)
      .def("does_intersect",
           [](const Segment2 &self, const Segment2 &s2) {
             return CGAL::do_intersect(self, s2);
           })
      .def("does_intersect",
           [](const Segment2 &self, const Point &p) {
             return CGAL::do_intersect(self, p);
           })
      .def("__str__", [](const Segment2 &self) {
        return fmt::format("[{}, {}]", point_to_string(self.source()),
                           point_to_string(self.target()));
      });

  // Polygons
  py::class_<Polygon2>(m, "Polygon", "A simple polygon in CGAL.")
      .def(py::init<>())
      .def(py::init([](const std::vector<Point> &vertices) {
        return std::make_unique<Polygon2>(vertices.begin(), vertices.end());
      }))
      .def("boundary",
           [](const Polygon2 &poly) {
             std::vector<Point> points;
             std::copy(poly.begin(), poly.end(), std::back_inserter(points));
             return points;
           })
      .def("is_simple", &Polygon2::is_simple)
      .def("contains",
           [](const Polygon2 &self, const Point &p) {
             return self.bounded_side(p) != CGAL::ON_UNBOUNDED_SIDE;
           })
      .def("contains",
           [](const Polygon2 &self, const Segment2 &s) {
             bool both_points_inside =
                 self.bounded_side(s.source()) != CGAL::ON_UNBOUNDED_SIDE &&
                 self.bounded_side(s.target()) != CGAL::ON_UNBOUNDED_SIDE;
             if (!both_points_inside) {
               return false;
             }
             for (auto it = self.edges_begin(); it != self.edges_end(); ++it) {
               if (CGAL::do_intersect(*it, s)) {
                 return false;
               }
             }
             return true;
           })
      .def("on_boundary",
           [](const Polygon2 &self, const Point &p) {
             return self.bounded_side(p) == CGAL::ON_BOUNDARY;
           })
      .def("area", [](const Polygon2 &poly) { return poly.area(); });

  // Convex hull
  m.def("compute_convex_hull", &compute_convex_hull,
        "Compute the convex hull of a set of points.");

  // Squared distance functions
  m.def("squared_distance", [](const Point &p1, const Point &p2) {
    return CGAL::squared_distance(p1, p2);
  });
  m.def("squared_distance", [](const Segment2 &s, const Point &p) {
    return CGAL::squared_distance(s, p);
  });
  m.def("squared_distance", [](const Point &p, const Segment2 &s) {
    return CGAL::squared_distance(p, s);
  });
  m.def("squared_distance", [](const Segment2 &s1, const Segment2 &s2) {
    return CGAL::squared_distance(s1, s2);
  });
  m.def("intersection_point", &intersection_point,
        "Compute the intersection point of two segments.");

  // VerificationGeometryHelper bindings
  py::class_<VerificationGeometryHelper>(
      m, "VerificationGeometryHelper",
      "An exact solution verifier using arrangements.")
      .def(py::init<>())
      .def("add_point", &VerificationGeometryHelper::add_point)
      .def("add_segment", &VerificationGeometryHelper::add_segment)
      .def("search_for_isolated_points",
           &VerificationGeometryHelper::search_for_isolated_points)
      .def("search_for_bad_edges",
           &VerificationGeometryHelper::search_for_bad_edges)
      .def("get_num_points", &VerificationGeometryHelper::get_num_points)
      .def("search_for_non_triangular_faces",
           &VerificationGeometryHelper::search_for_non_triangular_faces)
      .def("search_for_faces_with_holes",
           &VerificationGeometryHelper::search_for_faces_with_holes)
      .def("count_obtuse_triangles",
           &VerificationGeometryHelper::count_obtuse_triangles);

  // ConstrainedTriangulation bindings
  py::class_<ConstrainedTriangulation>(m, "ConstrainedTriangulation",
                                       "A constrained triangulation.")
      .def(py::init<>())
      .def("add_point", &ConstrainedTriangulation::add_point)
      .def("add_segment", &ConstrainedTriangulation::add_segment)
      .def("get_triangulation_edges",
           &ConstrainedTriangulation::get_triangulation_edges)
      .def("add_boundary", &ConstrainedTriangulation::add_boundary);

  // Duplicate check
  m.def("points_contain_duplicates", &points_contain_duplicates,
        "Check if a list of points contains duplicates.");

  // Triangulation check
  m.def("is_triangulation", &is_triangulation,
        "Check if a set of edges forms a triangulation of the given points.",
        py::arg("points"), py::arg("edges"), py::arg("verbose") = false);
  m.def("compute_triangles", &compute_triangles,
        "Compute all triangles formed by the given points and edges.");
  m.def("do_cross", &do_cross, "Check if two segments cross each other.");
}
