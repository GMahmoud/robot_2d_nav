#pragma once

#include <mutex>
#include <string>

#include <boost/thread.hpp>

#include <nav_grid/NavGrid.h>

#include "nav_costmap/Bounds.h"
#include "nav_costmap/Common.h"

namespace nav_costmap
{

using mutex_t = boost::recursive_mutex;

class CostmapInterface : public nav_grid::NavGrid< unsigned char >
{
public:
  static const unsigned char NO_INFORMATION = 255;
  static const unsigned char LETHAL_OBSTACLE = 254;
  static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
  static const unsigned char FREE_SPACE = 0;

  ///
  /// \brief Virtual Destructor
  ///
  virtual ~CostmapInterface() {}

  ///
  /// \brief  Initialization function for the CostmapInterface
  ///
  /// ROS parameters/topics are expected to be in the parent/name namespace.
  /// It is suggested that all NodeHandles in the riskmap use the parent
  /// NodeHandle's callback queue.
  ///
  /// \param  parent NodeHandle to derive other NodeHandles from
  /// \param  name The namespace for the riskmap
  /// \param  tf A pointer to a transform listener
  ///
  virtual void initialize(const ros::NodeHandle &parent, const std::string &name, TFListenerPtr tf) {}

  inline unsigned char getCost(unsigned int x, unsigned int y) { return getValue(x, y); }

  inline unsigned char getCost(const nav_grid::Index &index) { return getValue(index.x, index.y); }

  inline void setCost(unsigned int x, unsigned int y, unsigned char risk) { setValue(x, y, risk); }

  inline void setCost(const nav_grid::Index &index, unsigned char risk) { setValue(index, risk); }

  ///
  /// \brief Update the values in the riskmap
  ///
  /// Note that this method can throw RiskmapExceptions to indicate problems,
  /// like when it would be unsafe to navigate. e.g. If the riskmap expects
  /// laser data at a given rate, but laser data hasn't shown up in a while,
  /// this method might throw a RiskmapDataLagException.
  ///
  virtual void update() {}

  ///
  /// \brief Accessor for boost mutex
  ///
  virtual mutex_t *getMutex() = 0;

  ///
  /// \brief Flag to indicate whether this riskmap is able to track how much has
  /// changed
  ///
  virtual bool canTrackChanges() { return false; }

  ///
  /// \brief If canTrackChanges, get the bounding box for how much of the
  /// riskmap has changed
  ///
  /// Rather than querying based on time stamps (which can require an arbitrary
  /// amount of storage) we instead query based on a namespace. The return
  /// bounding box reports how much of the riskmap has changed since the last
  /// time this method was called with a particular namespace. If a namespace is
  /// new, then it returns a bounding box for the whole riskmap. The max values
  /// are inclusive.
  ///
  /// Example Sequence with a 5x5 riskmap: (results listed (min_x,min_y):(max_x,
  /// max_y)) 0) getChangeBounds("A") --> (0,0):(4,4) 1) getChangeBounds("B")
  /// --> (0,0):(4,4) 2) update cell 1, 1 3) getChangeBounds("C") -->
  /// (0,0):(4,4) 4) getChangeBounds("A") --> (1,1):(1,1) 5)
  /// getChangeBounds("A") --> (empty bounds)    (i.e. nothing was updated since
  /// last call) 6) updateCell 2, 4 7) getChangeBounds("A") --> (2,4):(2,4) 8)
  /// getChangeBounds("B") --> (1,1):(2,4)
  ///
  /// \param ns The namespace
  /// \return Updated bounds
  /// \throws std::runtime_error If canTrackChanges is false, the returned
  /// bounds would be meaningless
  ///
  virtual UIntBounds getChangeBounds(const std::string &ns)
  {
    if(!canTrackChanges())
    {
      throw std::runtime_error("You called 'getChangeBounds()' on a derived Riskmap type that is "
                               "not capable of "
                               "tracking changes (i.e. canTrackChanges() returns false). You "
                               "shouldn't do that.");
    }
    else
    {
      throw std::runtime_error("You called 'getChangeBounds()' on a derived Riskmap type that is "
                               "capable of tracking "
                               "changes but has not properly implemented this function. You should "
                               "yell at the author "
                               "of the derived Riskmap.");
    }
    return UIntBounds();
  }
};
} // namespace nav_costmap
