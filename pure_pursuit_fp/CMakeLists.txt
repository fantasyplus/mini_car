cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit_fp)
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

  #action
  actionlib_msgs
  nav_msgs
)

generate_dynamic_reconfigure_options(config/config.cfg) 

add_action_files(
  DIRECTORY action
  FILES PurePursuit.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES purepursuit_simulation
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf2 tf2_ros actionlib_msgs nav_msgs
#  DEPENDS system_lib
)


add_executable(pure_pursuit_server src/pure_pursuit_server.cpp)
target_link_libraries(pure_pursuit_server ${catkin_LIBRARIES})

add_executable(pure_pursuit_client src/pure_pursuit_client.cpp)
target_link_libraries(pure_pursuit_client ${catkin_LIBRARIES})

add_executable(pure_pursuit_fp src/pure_pursuit_fp.cpp)
target_link_libraries(pure_pursuit_fp ${catkin_LIBRARIES})


add_executable(pure_pursuit_fp_class src/pure_pursuit_fp_class.cpp)
target_link_libraries(pure_pursuit_fp_class ${catkin_LIBRARIES})