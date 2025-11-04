#pragma once

#include "cgal_types.h"
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace cgshop2026 {

// Define CGAL types for constrained triangulation
using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<
    Kernel, Tds, CGAL::No_constraint_intersection_tag>;

class ConstrainedTriangulation {
public:
  ConstrainedTriangulation();

  // Add a point to the triangulation and return its index
  int add_point(const Point &p);

  // Add a boundary made of points and verify if it is valid
  void add_boundary(const std::vector<int> &boundary);

  // Add a segment between two points in the triangulation
  void add_segment(const int i, const int j);

  // Get the edges of the triangulation, ignoring those outside the boundary
  std::vector<std::tuple<int, int>> get_triangulation_edges() const;

private:
  std::optional<Polygon2> boundary_; // Store the boundary polygon
  std::vector<CDT::Vertex_handle>
      points_; // Store the points in the triangulation
  std::unordered_map<CDT::Vertex_handle, int>
      point_to_index_; // Map points to their indices
  CDT cdt_;            // The constrained Delaunay triangulation
};

} // namespace cgshop2026
