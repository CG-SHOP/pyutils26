#pragma once

#include "cgal_types.h"
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <cmath>

namespace cgshop2026 {

/**
 * @brief Represents a 2D point with integral coordinates.
 * 
 * This class provides an efficient representation of CGAL Point_2 objects
 * that have integral coordinates, using native long integers for storage
 * and comparison operations. This optimization significantly improves
 * performance for hash-based lookups and comparisons.
 * 
 * @throws std::invalid_argument if constructed from a Point with non-integral coordinates.
 */
class IntegralPoint {
public:
  /**
   * @brief Construct an IntegralPoint from a CGAL Point.
   * 
   * @param p The CGAL point to convert.
   * @throws std::invalid_argument if the point has non-integral coordinates.
   */
  explicit IntegralPoint(const Point& p) {
    // Convert to double and check if coordinate is (close to) integral
    const double dx = CGAL::to_double(p.x());
    const double dy = CGAL::to_double(p.y());

    // Tolerance to account for floating conversion
    constexpr double eps = 1e-9;
    const double rx = std::round(dx);
    const double ry = std::round(dy);

    if (std::abs(dx - rx) > eps || std::abs(dy - ry) > eps) {
      throw std::invalid_argument(
          "IntegralPoint requires point with integral coordinates.");
    }

    x_ = static_cast<long>(rx);
    y_ = static_cast<long>(ry);
  }

  /**
   * @brief Equality comparison operator.
   */
  bool operator==(const IntegralPoint& other) const {
    return x_ == other.x_ && y_ == other.y_;
  }

  /**
   * @brief Inequality comparison operator.
   */
  bool operator!=(const IntegralPoint& other) const {
    return !(*this == other);
  }

  /**
   * @brief Lexicographic ordering: first by x, then by y.
   */
  bool operator<(const IntegralPoint& other) const {
    return (x_ < other.x_) || (x_ == other.x_ && y_ < other.y_);
  }

  /**
   * @brief Get the x-coordinate.
   */
  [[nodiscard]] long x() const { return x_; }

  /**
   * @brief Get the y-coordinate.
   */
  [[nodiscard]] long y() const { return y_; }

  /**
   * @brief Hash functor for use with unordered containers.
   */
  struct Hash {
    std::size_t operator()(const IntegralPoint& p) const noexcept {
      const std::size_t h1 = std::hash<long>{}(p.x());
      const std::size_t h2 = std::hash<long>{}(p.y());
      // XOR with bit shift provides good hash distribution
      return h1 ^ (h2 << 1);
    }
  };

private:
  long x_;  ///< X-coordinate
  long y_;  ///< Y-coordinate
};

/**
 * @brief Fast point-to-index lookup map using integral coordinates.
 * 
 * This class provides O(1) average-case lookup from CGAL Point objects
 * to their corresponding indices in a point array. It uses IntegralPoint
 * internally for efficient hashing and comparison.
 * 
 * @note All input points must have integral coordinates.
 */
class IntegralPointIndexMap {
public:
  /**
   * @brief Construct the index map from a vector of points.
   * 
   * @param points Vector of CGAL points with integral coordinates.
   * @throws std::invalid_argument if any point has non-integral coordinates.
   */
  explicit IntegralPointIndexMap(const std::vector<Point>& points) {
    point_to_index_.reserve(points.size());
    
    for (size_t i = 0; i < points.size(); ++i) {
      IntegralPoint ip(points[i]);
      point_to_index_.emplace(ip, static_cast<int>(i));
    }
  }

  /**
   * @brief Look up the index of a given integral point.
   * 
   * @param ip The integral point to look up.
   * @return The index of the point if found, otherwise std::nullopt.
   */
  [[nodiscard]] std::optional<int> get_index(const IntegralPoint& ip) const {
    const auto it = point_to_index_.find(ip);
    
    if (it != point_to_index_.end()) {
      return it->second;
    }
    
    return std::nullopt;
  }

  /**
   * @brief Look up the index of a given CGAL point (convenience overload).
   * 
   * @param p The CGAL point to look up.
   * @return The index of the point if found, otherwise std::nullopt.
   * @throws std::invalid_argument if the point has non-integral coordinates.
   */
  [[nodiscard]] std::optional<int> get_index(const Point& p) const {
    return get_index(IntegralPoint(p));
  }

  /**
   * @brief Get the number of points in the map.
   */
  [[nodiscard]] size_t size() const noexcept {
    return point_to_index_.size();
  }

  /**
   * @brief Check if the map is empty.
   */
  [[nodiscard]] bool empty() const noexcept {
    return point_to_index_.empty();
  }

 /**
  * @brief Check if point is in the map.
  */
  [[nodiscard]] bool contains(const IntegralPoint& ip) const noexcept {
    return point_to_index_.find(ip) != point_to_index_.end();
  }
  
  [[nodiscard]] bool contains(const Point& p) const {
    return contains(IntegralPoint(p));
  }

private:
  std::unordered_map<IntegralPoint, int, IntegralPoint::Hash> point_to_index_;
};

} // namespace cgshop2026