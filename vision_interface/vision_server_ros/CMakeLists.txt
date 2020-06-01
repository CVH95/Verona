cmake_minimum_required(VERSION 2.8.3)
project(vision_server_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  roslib
  cv_bridge
  sensor_msgs
  geometry_msgs
  tf2_ros
  eigen_conversions
  vision_lib
  vision_lib_msgs
  perception_servoing
)

find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES vision_server_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Libraries
add_library(vision_server_ros src/vision_server_ros.cpp)
target_link_libraries(vision_server_ros ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

# Nodes
add_executable(vision_server_ros_node src/vision_server_ros_node.cpp)
target_link_libraries(vision_server_ros_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} vision_server_ros)

install(TARGETS vision_server_ros vision_server_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)