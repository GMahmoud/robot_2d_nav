cmake_minimum_required(VERSION 2.8.3)
project(nav_costmap)

set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
    COMPONENTS nav_grid tf2_ros costmap_2d
)

catkin_package(
    CATKIN_DEPENDS nav_grid  tf2_ros costmap_2d
    INCLUDE_DIRS include
    LIBRARIES CostmapAdaptation
)

add_library(CostmapAdaptation src/CostmapAdaptation.cpp)
target_link_libraries(
    CostmapAdaptation ${catkin_LIBRARIES}
)

include_directories(
     ${catkin_INCLUDE_DIRS}
     include
)


add_executable(test src/test.cpp)
target_link_libraries(test 
PUBLIC
${catkin_LIBRARIES} 
CostmapAdaptation
)