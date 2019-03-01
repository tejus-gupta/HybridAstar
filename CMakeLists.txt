cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)

## C++11
set( CMAKE_CXX_STANDARD 11 )


find_package(catkin REQUIRED COMPONENTS 
   tf
   tf2
   cv_bridge
   image_transport
   rospy
   roscpp
   nav_msgs
   std_msgs
   geometry_msgs
)
find_package (Threads)



## OPEN MOTION PLANNING LIBRARY
find_package(OMPL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)


## Declare a catkin package
catkin_package()

## Linking 
include_directories(include ${OMPL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(Test_ROS src/Test.cpp )
target_link_libraries(Test_ROS ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})