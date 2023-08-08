cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit_fp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(config/config.cfg) 

catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES purepursuit_simulation
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit_fp src/pure_pursuit_fp.cpp)
target_link_libraries(pure_pursuit_fp ${catkin_LIBRARIES})
