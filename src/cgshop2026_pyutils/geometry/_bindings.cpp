// pybind11
#include <pybind11/operators.h> // To define operator overloading
#include <pybind11/pybind11.h>  // Basic pybind11 functionality
#include <pybind11/stl.h>       // Automatic conversion of vectors

// Local headers
#include "cgal_types.h"
#include "constrained_triangulation.h"
#include "geometry_operations.h"
#include "number_parsing.h"

// CGAL for distance functions
#include <CGAL/squared_distance_2.h>

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
