#pragma once

#include <algorithm>
#include <limits>
#include <string>

namespace nav_costmap {

///
/// \brief Templatized class that represents a two dimensional bounds with
/// ranges [minX, maxX] [minY, maxY] inclusive
///
template <typename NumericType>
struct GenericBounds {
 public:
  ///
  /// \brief Constructor for an empty bounds
  ///
  GenericBounds() { reset(); }

  ///
  /// \brief Constructor for a non-empty initial bounds
  /// \param x0 Initial min x
  /// \param y0 Initial min y
  /// \param x1 Initial max x
  /// \param y1 Initial max y
  ///
  GenericBounds(NumericType x0, NumericType y0, NumericType x1, NumericType y1)
      : _min_x(x0), _min_y(y0), _max_x(x1), _max_y(y1) {}

  ///
  /// \brief Reset the bounds to be empty
  ///
  void reset() {
    _min_x = _min_y = std::numeric_limits<NumericType>::max();
    _max_x = _max_y = std::numeric_limits<NumericType>::lowest();
  }

  ///
  /// \brief Update the bounds to include the point (x, y)
  ///
  void touch(NumericType x, NumericType y) {
    _min_x = std::min(x, _min_x);
    _min_y = std::min(y, _min_y);
    _max_x = std::max(x, _max_x);
    _max_y = std::max(y, _max_y);
  }

  ///
  /// \brief Update the bounds to include points (x0, y0) and (x1, y1)
  /// \param x0 smaller of two x values
  /// \param y0 smaller of two y values
  /// \param x1 larger of two x values
  /// \param y1 larger of two y values
  ///
  void update(NumericType x0, NumericType y0, NumericType x1, NumericType y1) {
    _min_x = std::min(x0, _min_x);
    _min_y = std::min(y0, _min_y);
    _max_x = std::max(x1, _max_x);
    _max_y = std::max(y1, _max_y);
  }

  ///
  /// \brief Update the bounds to include the entirety of another bounds object
  /// \param other Another bounds object
  ///
  void merge(const GenericBounds<NumericType>& other) {
    update(other._min_x, other._min_y, other._max_x, other._max_y);
  }

  ///
  /// \brief Returns true if the range is empty
  ///
  bool isEmpty() const { return _min_x > _max_x && _min_y > _max_y; }

  ///
  /// \brief Returns a string representation of the bounds
  ///
  std::string toString() const {
    if (!isEmpty()) {
      return "(" + std::to_string(_min_x) + "," + std::to_string(_min_y) + "):(" +
             std::to_string(_max_x) + "," + std::to_string(_max_y) + ")";
    } else {
      return "(empty bounds)";
    }
  }

  NumericType getMinX() const { return _min_x; }
  NumericType getMinY() const { return _min_y; }
  NumericType getMaxX() const { return _max_x; }
  NumericType getMaxY() const { return _max_y; }

 protected:
  NumericType _min_x, _min_y, _max_x, _max_y;
};

using Bounds = GenericBounds<double>;

class UIntBounds : public GenericBounds<unsigned int> {
 public:
  using GenericBounds<unsigned int>::GenericBounds;
  unsigned int getWidth() const { return _max_x - _min_x + 1; }
  unsigned int getHeight() const { return _max_y - _min_y + 1; }
};

}  // namespace nav_costmap
