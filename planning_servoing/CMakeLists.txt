cmake_minimum_required(VERSION 2.8.3)
project(planning_servoing)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  roslib
  std_msgs
  gazebo_msgs
  geometry_msgs
  vision_lib_msgs
  tf2_ros
  eigen_conversions
  moveit_core
  moveit_commander
  moveit_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group
  moveit_planners_ompl
  moveit_ros_visualization
  moveit_fake_controller_manager
  moveit_simple_controller_manager
  moveit_visual_tools
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES planning_servoing
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Libraries
add_library(planning_servoing src/planning_servoing.cpp)
target_link_libraries(planning_servoing ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

# Nodes
add_executable(marker_trajectory_node nodes/marker_trajectory_node.cpp)
target_link_libraries(marker_trajectory_node ${catkin_LIBRARIES} )

add_executable(planning_servoing_node nodes/planning_servoing_node.cpp)
target_link_libraries(planning_servoing_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} planning_servoing)

install(TARGETS marker_trajectory_node planning_servoing
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)