cmake_minimum_required(VERSION 2.8.3)
project(sawyer_humcpp)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#find_package(Armadillo REQUIRED)
#include_directories(${ARMADILLO_INCLUDE_DIRS})

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs visualization_msgs genmsg message_generation
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  intera_core_msgs
  intera_motion_msgs
  intera_interface)

add_message_files(FILES mdasys.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
   # INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs genmsg visualization_msgs message_runtime
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  intera_core_msgs
  intera_motion_msgs
  intera_interface
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
add_executable(impede_test src/impede_test.cpp)
add_dependencies(impede_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(impede_test ${catkin_LIBRARIES} armadillo)

#add_executable(visualizer src/visualizer.cpp)
#add_dependencies(visualizer ${catkin_EXPORTED_TARGETS})
#target_link_libraries(visualizer ${catkin_LIBRARIES})

