#pragma once

#include "cgal_types.h"
#include <string>

namespace cgshop2026 {

// Function to convert Kernel::FT to rational string
std::string to_rational_string(const Kernel::FT &x);

// Exact conversion of a long to CGAL's Field Type (FT)
Kernel::FT to_exact(std::int64_t x);

// Convert string to exact number (Kernel::FT), handling rational numbers
Kernel::FT str_to_exact(std::string number);

} // namespace cgshop2026
