

#pragma once

#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>

#include "nav_costmap/CostmapInterface.h"

namespace nav_costmap
{

nav_grid::NavGridInfo infoFromCostmap(std::shared_ptr< costmap_2d::Costmap2DROS > costmapROS);

class CostmapAdaptation : public CostmapInterface
{
public:
  ///
  /// \brief Deconstructor for possibly freeing the costmap_ros_ object
  ///
  virtual ~CostmapAdaptation();

  ///
  /// \brief Initialize from an existing Costmap2DROS object
  /// \param costmap_ros A Costmap2DROS object
  /// \ param destruction Whether to free the costmap_ros object when this class
  /// is destroyed
  ///
  void initialize(std::shared_ptr< costmap_2d::Costmap2DROS > costmapROS, bool destructible = false);

  // Costmap Interface
  void initialize(const ros::NodeHandle &parent, const std::string &name, TFListenerPtr tf) override;

  mutex_t *getMutex() override;

  void update() override;

  /// NavGrid Interface
  void reset() override;
  void setValue(unsigned int x, unsigned int y, const unsigned char &value) override;
  unsigned char getValue(unsigned int x, unsigned int y) const override;

  void setInfo(const nav_grid::NavGridInfo &info) override;

  void updateInfo(const nav_grid::NavGridInfo &info) override;

  /// Index Conversion
  unsigned int getIndex(unsigned int x, unsigned int y) const;

  /// Get Costmap Pointer for Backwards Compatibility
  std::shared_ptr< costmap_2d::Costmap2DROS > getCostmap2DROS() const { return _costmapROS; }

protected:
  std::shared_ptr< costmap_2d::Costmap2DROS > _costmapROS;
  costmap_2d::Costmap2D *_costmap;
  bool _destructible;
};

using CostmapAdaptationPtr = std::shared_ptr< CostmapAdaptation >;

} // namespace nav_costmap
