cmake_minimum_required(VERSION 2.8.3)
project(vision_detection)

add_definitions(-std=c++11 )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs genmsg geometry_msgs)
find_package(OpenCV REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Executables
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(vision src/vision_node.cpp src/classifier.cpp)
target_link_libraries(vision ${catkin_LIBRARIES} ${OpenCV_LIBS})
