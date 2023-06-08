cmake_minimum_required(VERSION 3.0.2)
project(racebot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teb_stop_node src/stop.cpp)

target_link_libraries(teb_stop_node
  ${catkin_LIBRARIES}
)