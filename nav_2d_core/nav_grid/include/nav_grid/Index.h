#pragma once

#include <string>

namespace nav_grid {
///
/// \brief A simple pair of x/y coordinates
///
template <typename NumericType>
struct GenericIndex {
  NumericType x, y;
  explicit GenericIndex(const NumericType& x = 0, const NumericType& y = 0)
      : x(x), y(y) {}

  ///
  /// \brief comparison operator that requires equal x and y
  ///
  bool operator==(const GenericIndex& other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const GenericIndex& other) const {
    return !operator==(other);
  }

  ///
  /// \brief less than operator so object can be used in sets
  ///
  bool operator<(const GenericIndex& other) const {
    return x < other.x || (x == other.x && y < other.y);
  }

  // Derived Comparators
  bool operator>(const GenericIndex& other) const { return other < *this; }
  bool operator<=(const GenericIndex& other) const { return !(*this > other); }
  bool operator>=(const GenericIndex& other) const { return !(*this < other); }

  ///
  /// \brief String representation of this object
  ///
  std::string toString() const {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
  }
};

template <typename NumericType>
inline std::ostream& operator<<(std::ostream& stream,
                                const GenericIndex<NumericType>& index) {
  stream << index.toString();
  return stream;
}

using SignedIndex = GenericIndex<int>;
using Index = GenericIndex<unsigned int>;

}  // namespace uavia_nav_grid
