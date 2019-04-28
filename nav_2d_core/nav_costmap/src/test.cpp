#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// #include <locomotor/executor.h>

#include "nav_costmap/CostmapAdaptation.h"

nav_costmap::CostmapAdaptationPtr costmap;
std::shared_ptr< tf2_ros::Buffer > tf;
std::shared_ptr< tf2_ros::TransformListener > tf2_listener;

void init(std::shared_ptr< ros::NodeHandle > node)
{
  tf = std::make_shared< tf2_ros::Buffer >();
  tf2_listener = std::make_shared< tf2_ros::TransformListener >(*tf);
  costmap = nav_costmap::CostmapAdaptationPtr(new nav_costmap::CostmapAdaptation());
  costmap->initialize(*node, "global_costmap", tf);
}

// --------------------------------------------------------------------
// void callbackCone(const std_msgs::EmptyConstPtr &msg) {}
// ----------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "costma_test");
  std::shared_ptr< ros::NodeHandle > node = std::shared_ptr< ros::NodeHandle >(new ros::NodeHandle("~"));

  init(node);
  //   ros::Subscriber sub = n.subscribe<std_msgs::Empty>("/test", 5,
  //   &callback); ros::Subscriber subCone =
  //   n.subscribe<std_msgs::Empty>("/test2", 5, &callbackCone);

  ros::spin();

  return 0;
}
