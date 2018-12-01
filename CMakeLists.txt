cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)
set( CMAKE_CXX_STANDARD 11 )

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  costmap_2d
  costmap_converter
  std_msgs
  tf
  tf2
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable( Test src/Test.cpp)

target_link_libraries(Test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


