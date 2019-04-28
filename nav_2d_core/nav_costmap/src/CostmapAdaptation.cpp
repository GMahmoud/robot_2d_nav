#include "nav_costmap/CostmapAdaptation.h"

namespace nav_costmap
{

// ----------------------------------------------------------------------------

nav_grid::NavGridInfo infoFromCostmap(std::shared_ptr< costmap_2d::Costmap2DROS > costmapROS)
{
  ROS_WARN("Hola from infoFromCostmap");
  nav_grid::NavGridInfo info;
  costmap_2d::Costmap2D *costmap = costmapROS->getCostmap();
  info.width = costmap->getSizeInCellsX();
  info.height = costmap->getSizeInCellsY();
  info.resolution = costmap->getResolution();
  info.frame_id = costmapROS->getGlobalFrameID();
  info.origin_x = costmap->getOriginX();
  info.origin_y = costmap->getOriginY();
  return info;
}

// ----------------------------------------------------------------------------

CostmapAdaptation::~CostmapAdaptation()
{
  if(_destructible)
  {
    delete _costmapROS.get();
  }
}

// ----------------------------------------------------------------------------

void CostmapAdaptation::initialize(std::shared_ptr< costmap_2d::Costmap2DROS > costmapROS, bool destructible)
{
  ROS_WARN("Hola from init");
  _costmapROS = costmapROS;
  _destructible = destructible;
  _info = infoFromCostmap(_costmapROS);
  _costmap = _costmapROS->getCostmap();
}

// ----------------------------------------------------------------------------

void CostmapAdaptation::initialize(const ros::NodeHandle &parent, const std::string &name, TFListenerPtr tf) { initialize(std::shared_ptr< costmap_2d::Costmap2DROS >(new costmap_2d::Costmap2DROS(name, *tf)), true); }

// ----------------------------------------------------------------------------

mutex_t *CostmapAdaptation::getMutex() { return _costmap->getMutex(); }

// ----------------------------------------------------------------------------

void CostmapAdaptation::update()
{
  ROS_WARN("Hola from update");

  _info = infoFromCostmap(_costmapROS);
  if(!_costmapROS->isCurrent())
    throw std::runtime_error("Costmap2DROS is out ...");
}

// ----------------------------------------------------------------------------

void CostmapAdaptation::reset()
{
  ROS_WARN("Hola from reset");

  _costmap->resetMap(0, 0, _costmap->getSizeInCellsX(), _costmap->getSizeInCellsY());
}

// ----------------------------------------------------------------------------

void CostmapAdaptation::setValue(unsigned int x, unsigned int y, const unsigned char &value)
{
  ROS_WARN("Hola from setValue");

  _costmap->setCost(x, y, value);
}

// ----------------------------------------------------------------------------

unsigned char CostmapAdaptation::getValue(unsigned int x, unsigned int y) const
{
  unsigned int index = _costmap->getIndex(x, y);
  ROS_WARN("Hola from getValue");

  return _costmap->getCharMap()[index];
}

// ----------------------------------------------------------------------------

void CostmapAdaptation::setInfo(const nav_grid::NavGridInfo &info)
{
  ROS_WARN("Hola from setInfo");

  throw std::runtime_error("setInfo not implemented yet");
}

// ----------------------------------------------------------------------------

void CostmapAdaptation::updateInfo(const nav_grid::NavGridInfo &info)
{
  ROS_WARN("Hola from updateInfo");

  _costmap->updateOrigin(info.origin_x, info.origin_y);
}

// ----------------------------------------------------------------------------

unsigned int CostmapAdaptation::getIndex(unsigned int x, unsigned int y) const
{
  ROS_WARN("Hola from getIndex");

  return y * _info.width + x;
}

} // namespace nav_costamp
