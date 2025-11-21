#pragma once

#include "cgal_types.h"
#include <cstdint>
#include <fmt/core.h>
#include <string>

namespace cgshop2026 {

// ============================================================================
// String conversion utilities
// ============================================================================

/**
 * @brief Convert a CGAL point to a human-readable string representation.
 * @param p The point to convert.
 * @return String in format "(x, y)" with double-precision coordinates.
 */
std::string point_to_string(const Point &p);

/**
 * @brief Convert a field type value to its exact rational string
 * representation.
 * @param x The field type value to convert.
 * @return String representation of the exact rational value.
 */
std::string to_rational_string(const Kernel::FT &x);

// ============================================================================
// Exact number conversion utilities
// ============================================================================

/**
 * @brief Convert a 64-bit integer to CGAL's exact field type.
 * @param x The integer to convert.
 * @return Exact representation as Kernel::FT.
 */
Kernel::FT to_exact(std::int64_t x);

/**
 * @brief Parse a string representation of a number into CGAL's exact field
 * type.
 *
 * Supports both integers and rational numbers (e.g., "123", "-456", "22/7").
 * Whitespace is ignored. Leading zeros and plus signs are stripped.
 *
 * @param number String representation of the number.
 * @return Exact representation as Kernel::FT.
 * @throws std::runtime_error if the string contains invalid characters or
 * format.
 */
Kernel::FT str_to_exact(std::string number);

// ============================================================================
// Comparators and hash functions
// ============================================================================

/**
 * @brief Lexicographic comparison for CGAL points (x then y).
 *
 * This comparator is robust with CGAL's exact number types and suitable
 * for use with ordered containers like std::map or std::set.
 */
struct LessPointXY {
  bool operator()(const Point &a, const Point &b) const;
};

/**
 * @brief Hash function for std::tuple<int, int>.
 *
 * Suitable for use with unordered containers like std::unordered_set
 * or std::unordered_map when storing edge pairs or similar tuples.
 */
struct TupleHash {
  std::size_t operator()(const std::tuple<int, int> &t) const;
};

} // namespace cgshop2026
