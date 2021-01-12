cmake_minimum_required(VERSION 2.8.3)
project(cartbot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  std_msgs
  tf
  sensor_msgs
  laser_geometry
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cartbot
#  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  main src/main.cpp
)

add_executable(
  observation src/observation.cpp
)

add_executable(
  listener_sample src/listener_sample.cpp
)

target_link_libraries(main
  ${catkin_LIBRARIES}
)

target_link_libraries(observation
  ${catkin_LIBRARIES}
)

target_link_libraries(listener_sample
  ${catkin_LIBRARIES}
)