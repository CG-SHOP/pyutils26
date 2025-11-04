#include "number_parsing.h"
#include <CGAL/Exact_integer.h>
#include <CGAL/Exact_rational.h>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>

namespace cgshop2026 {

// Function to convert Kernel::FT to rational string
std::string to_rational_string(const Kernel::FT &x) {
  auto exact_value = CGAL::exact(x);
  std::string exact_str = exact_value.str();
  return exact_str;
}

// Exact conversion of a long to CGAL's Field Type (FT)
Kernel::FT to_exact(std::int64_t x) {
  double lo32 = x & 0xffff'ffff;
  double hi32 = static_cast<double>(x >> 32) * 4294967296.0;
  return Kernel::FT(hi32) + Kernel::FT(lo32);
}

// Helper function to convert integer string to exact rational
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

// Remove whitespace from a number string
static void remove_whitespace(std::string &number) {
  // restricted whitespace detection;
  // cannot use std::isspace (negative chars cause UB)
  auto is_ws = [](char c) -> bool {
    return c == ' ' || c == '\t' || c == '\n';
  };
  number.erase(std::remove_if(number.begin(), number.end(), is_ws),
               number.end());
}

// Check if all characters in the number string are allowed
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

// Check that negative sign is in valid position
static void check_sign(const std::string &number) {
  if (number.empty())
    return;
  if (!std::all_of(number.begin() + 1, number.end(),
                   [](char c) { return std::isdigit(c); })) {
    throw std::runtime_error(
        "Negative sign character '-' in invalid position in number string.");
  }
}

// Convert integer string to exact number with validation
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

} // namespace cgshop2026
