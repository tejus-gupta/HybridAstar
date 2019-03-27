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

## Creating executable for test:
add_executable(test_simple src/test.cpp src/Planner.cpp src/GUI.cpp src/Heuristic.cpp src/Map.cpp src/Vehicle.cpp)
target_link_libraries(test_simple ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(test_simple ${catkin_EXPORTED_TARGETS})

## Creating executable for benchmarks:
add_executable(test_benchmark src/test_benchmark.cpp src/Planner.cpp src/GUI.cpp src/Heuristic.cpp src/Map.cpp src/Vehicle.cpp)
target_link_libraries(test_benchmark ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(test_benchmark ${catkin_EXPORTED_TARGETS})

## Creating executable for ros-integrated test:
add_executable(test_ros src/test_ros.cpp src/Planner.cpp src/GUI.cpp src/Heuristic.cpp src/Map.cpp src/Vehicle.cpp)
target_link_libraries(test_ros ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(test_ros ${catkin_EXPORTED_TARGETS})


## Creating executable for Velocity Publisher:
add_executable(velocity_publisher publisher/velocity_publisher.cpp )
target_link_libraries(velocity_publisher ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(velocity_publisher ${catkin_EXPORTED_TARGETS})

## Creating executable for Obstacle Publisher:
add_executable(obstacle_publisher publisher/obstacle_publisher.cpp )
target_link_libraries(obstacle_publisher ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(obstacle_publisher ${catkin_EXPORTED_TARGETS})