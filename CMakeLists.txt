cmake_minimum_required(VERSION 3.0.2)
project(transform_pointcloud)

add_compile_options(-std=c++17)

set(ROS_DEPS
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf
  roscpp
  laser_geometry
  cob_scan_unifier
)

find_package(
  catkin REQUIRED
  ${ROS_DEPS}
)

catkin_package(
  CATKIN_DEPENDS
  ${ROS_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp launch/${PROJECT_NAME}.launch)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})