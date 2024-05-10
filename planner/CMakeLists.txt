cmake_minimum_required(VERSION 3.0.2)
project(sea_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  grid_map_core
  grid_map_msgs
  grid_map_pcl
  grid_map_ros
  pcl_ros
  roscpp
  rospy
  message_generation
)

find_package(PCL REQUIRED QUIET)

add_message_files(
  FILES
  Edge.msg
  Graph.msg
  Vertex.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  get_nearest.srv
  get_state.srv
  get_path.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
  LIBRARIES sea_planner
  CATKIN_DEPENDS geometry_msgs nav_msgs grid_map_core grid_map_msgs grid_map_pcl grid_map_ros roscpp rospy message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_node 
src/sea_planner_node.cpp 
src/rtrrt.cpp 
src/rtrrtp.cpp 
src/utility.cpp
src/kd_search/kd_search.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(graph_planner_node 
src/graph_planner/graph_planner_node.cpp 
src/graph_planner/graph_planner.cpp 
src/utility.cpp
src/kd_search/kd_search.cpp)
add_dependencies(graph_planner_node ${PROJECT_NAME}_gencpp)
target_link_libraries(graph_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(graph_visualization_node src/graph_planner/graph_visualization_node.cpp)
add_dependencies(graph_visualization_node ${PROJECT_NAME}_gencpp)
target_link_libraries(graph_visualization_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

