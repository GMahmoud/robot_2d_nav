#pragma once

#include <string>

namespace nav_grid {
/**
 * @struct NavGridInfo
 * This class defines a way to discretize a finite section of the world into a
 * grid. It contains similar information to the ROS msg nav_msgs/MapMetaData
 * (aka the info field of nav_msgs/OccupancyGrid) except the map_load_time is
 * removed, the geometry is simplified from a Pose to xy coordinates, and the
 * frame_id is added.
 */
struct NavGridInfo {
  /// TODO
  unsigned int width = 0;
  /// TODO  
  unsigned int height = 0;
  /// TODO  
  double resolution = 1.0;
  
  /// TODO  
  std::string frame_id = "map";

  /// TODO
  /// The origin defines the coordinates of minimum corner of cell (0,0) in the
  /// grid
  double origin_x = 0.0;
  /// TODO  
  double origin_y = 0.0;

  /// \brief comparison operator that requires all fields are equal
  ///
  bool operator==(const NavGridInfo& info) const {
    return width == info.width && height == info.height &&
           resolution == info.resolution && origin_x == info.origin_x &&
           origin_y == info.origin_y && frame_id == info.frame_id;
  }

  bool operator!=(const NavGridInfo& info) const { return !operator==(info); }

  /// \brief String representation of this object
  ///
  std::string toString() const {
    return std::to_string(width) + "x" + std::to_string(height) + " (" +
           std::to_string(resolution) + "res) " + frame_id + " " +
           std::to_string(origin_x) + " " + std::to_string(origin_y);
  }
};

inline std::ostream& operator<<(std::ostream& stream, const NavGridInfo& info) {
  stream << info.toString();
  return stream;
}

}  // namespace uavia_nav_grid
