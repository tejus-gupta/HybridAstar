cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)

## C++11
set( CMAKE_CXX_STANDARD 11 )


find_package(catkin REQUIRED COMPONENTS 
   tf
   tf2
   rospy
   roscpp
   nav_msgs
   std_msgs
   geometry_msgs
   message_generation
)
find_package (Threads)


## OPEN MOTION PLANNING LIBRARY
find_package(OMPL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)


add_message_files(
   FILES
   polygon.msg
   polygonArray.msg
 )

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

## Declare a catkin package
catkin_package(
CATKIN_DEPENDS message_runtime
)

## Linking 
include_directories(include ${OMPL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Creating executable for Hybrid Astar Planner:
add_executable(hybrid_astar src/Test.cpp )
target_link_libraries(hybrid_astar ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(hybrid_astar ${catkin_EXPORTED_TARGETS})

## Creating executable for Velocity Publisher:
add_executable(velocity_publisher velocity_publisher/velocity_publisher.cpp )
target_link_libraries(velocity_publisher ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(velocity_publisher ${catkin_EXPORTED_TARGETS})