cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)
add_compile_options(-std=c++11 -o2 -g)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(config/config.cfg) 


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES purepursuit_simulation
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf2 tf2_ros nav_msgs
#  DEPENDS system_lib
)

add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})
