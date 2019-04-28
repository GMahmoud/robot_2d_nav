
#pragma once

#include <string>

#include "nav_grid/Index.h"
#include "nav_grid/NavGridInfo.h"

namespace nav_grid {
///
/// \brief NavGrid
/// This class is a spiritual successor to the costmap_2d::Costmap2D class, with
/// the key differences being that the datatype and data storage methods are not
/// specified, and the frame_id is specified.
///
/// The templatized nature of the class allows you to store whatever you like at
/// each grid location, including unsigned chars if emulating Costmap2D or
/// floating point numbers if emulating the grid_map package, or whatever else.
///
/// Getting data from the grid can be done either through the getValue methods
/// or the parenthetical operators (which call getValue internally).
/// Implementing classes must implement getValue. x = grid(0, 0) +
/// grid.getValue(0, 1);
///
/// Writing data to the grid must be done through the setValue method (which
/// implementing classes must implement) grid.setValue(0, 0, x);
///
/// You can also use nav_grid::Index objects
/// nav_grid::Index index(0, 0);
/// x = grid(index) + grid.getValue(index);
/// index.y = 3;
/// grid.setCost(index, x);
/// The Index methods also internally call setValue/getValue
///
/// The geometry of the grid is specified by the NavGridInfo. Borrowing an idea
/// from the grid_map package, two separate methods are defined for changing the
/// info. setInfo will change the info without changing the grid values.
/// updateInfo will change the info while trying to preserve the contents of the
/// grid.
///
/// The final component is a collection of methods inspired by Costmap2D for
/// converting coordinates of different types.
///
template <typename T>
class NavGrid {
 public:
  explicit NavGrid(const T defaultValue = T{}) : _defaultValue(defaultValue) {}

  ///
  /// \brief Reset the contents of the grid
  ////
  virtual void reset() = 0;

  ///
  /// \brief get the value of the grid at (x,y)
  /// \param x[in] Valid x coordinate
  /// \param y[in] Valid y coordinate
  /// \return value at (x,y)
  ////
  virtual T getValue(unsigned int x, unsigned int y) const = 0;

  ///
  /// \brief set the value of the grid at (x,y)
  /// \param x[in] Valid x coordinate
  /// \param y[in] Valid y coordinate
  /// \param value[in] New Value
  ////
  virtual void setValue(unsigned int x, unsigned int y, const T& value) = 0;

  /// \name Convenience Aliases
  /// Note: You may not be able to use these unless your deriving class declares
  /// using NavGrid<T>::operator() or using NavGrid<T>::getValue

  T getValue(const Index& index) { return getValue(index.x, index.y); }
  T operator()(unsigned int x, unsigned int y) const { return getValue(x, y); }
  T operator()(const Index& index) const {
    return getValue(index.x, index.y);
  }
  void setValue(const Index& index, const T& value) {
    setValue(index.x, index.y, value);
  }

  ///
  /// \brief Change the info while attempting to keep the values associated with
  /// the grid coordinates
  /// \param[in] new_info New grid info
  ////
  virtual void setInfo(const NavGridInfo& info) = 0;

  ///
  /// \brief Change the info while attempting to keep the values associated with
  /// the world coordinates
  ///
  /// For example, if the only change to the info is to the origin's x
  /// coordinate (increasing by an amount equal to the resolution), then all the
  /// values should be shifted one grid cell to the left.
  ///
  /// \param[in] new_info New grid info
  ////
  virtual void updateInfo(const NavGridInfo& info) { setInfo(info); }

  inline NavGridInfo getInfo() const { return _info; }

  ///
  /// \brief Set the default value
  /// \param[in] new_value New Default Value
  ///
  void setDefaultValue(const T newValue) { _defaultValue = newValue; }

  inline unsigned int getWidth() const { return _info.width; }
  inline unsigned int getHeight() const { return _info.height; }
  inline double getResolution() const { return _info.resolution; }
  inline std::string getFrameId() const { return _info.frame_id; }
  inline double getOriginX() const { return _info.origin_x; }
  inline double getOriginY() const { return _info.origin_y; }

 protected:
  NavGridInfo _info;
  T _defaultValue;
};

}  // namespace uavia_nav_grid
