#include "constrained_triangulation.h"
#include <stdexcept>

namespace cgshop2026 {

ConstrainedTriangulation::ConstrainedTriangulation() : cdt_() {}

// Add a point to the triangulation and return its index
int ConstrainedTriangulation::add_point(const Point &p) {
  auto index = points_.size();
  auto v = cdt_.insert(p);
  points_.push_back(v);
  point_to_index_[v] = index;
  return static_cast<int>(index);
}

// Add a boundary made of points and verify if it is valid
void ConstrainedTriangulation::add_boundary(const std::vector<int> &boundary) {
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
void ConstrainedTriangulation::add_segment(const int i, const int j) {
  auto p1 = points_[i];
  auto p2 = points_[j];
  cdt_.insert_constraint(p1, p2);
}

// Get the edges of the triangulation, ignoring those outside the boundary
std::vector<std::tuple<int, int>>
ConstrainedTriangulation::get_triangulation_edges() const {
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

} // namespace cgshop2026
