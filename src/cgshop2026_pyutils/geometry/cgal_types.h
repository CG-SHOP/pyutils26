#pragma once

// CGAL includes
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

// Standard library
#include <string>
#include <tuple>

// fmt
#include <fmt/core.h>

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

// Hash function for std::tuple<int, int> to use in unordered_set
struct TupleHash {
  std::size_t operator()(const std::tuple<int, int>& t) const {
    auto h1 = std::hash<int>{}(std::get<0>(t));
    auto h2 = std::hash<int>{}(std::get<1>(t));
    return h1 ^ (h2 << 1);
  }
};

// Less comparator that's robust with CGAL number types
struct LessPointXY {
  bool operator()(const Point& a, const Point& b) const {
    if (a.x() < b.x()) return true;
    if (b.x() < a.x()) return false;
    return a.y() < b.y();
  }
};

// Utility function to convert a point to a string
inline std::string point_to_string(const Point &p) {
  return fmt::format("({}, {})", CGAL::to_double(p.x()),
                     CGAL::to_double(p.y()));
}

} // namespace cgshop2026
