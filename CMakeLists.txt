cmake_minimum_required(VERSION 2.8.3)
project(sawyer_humcpp)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#find_package(Armadillo REQUIRED)
#include_directories(${ARMADILLO_INCLUDE_DIRS})
#find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
#include_directories(${ARMADILLO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OpenMP_INCLUDE_DIRS})

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
  intera_interface
  )

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

add_executable(draw_free src/draw_free.cpp)
add_dependencies(draw_free ${catkin_EXPORTED_TARGETS})
target_link_libraries(draw_free ${catkin_LIBRARIES} armadillo ${OpenCV_LIBS} ${OpenMP_LIBS})

#add_executable(motion_test src/motion_impedance.cpp)
#add_dependencies(motion_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(motion_test ${catkin_LIBRARIES} armadillo)

add_executable(gohome src/gohome.cpp)
add_dependencies(gohome ${catkin_EXPORTED_TARGETS})
target_link_libraries(gohome ${catkin_LIBRARIES})

add_executable(circle src/circle_impedance.cpp)
add_dependencies(circle ${catkin_EXPORTED_TARGETS})
target_link_libraries(circle ${catkin_LIBRARIES})

add_executable(walls src/virtual_fixture.cpp)
add_dependencies(walls ${catkin_EXPORTED_TARGETS})
target_link_libraries(walls ${catkin_LIBRARIES})

add_executable(applewalls src/apple_walls.cpp)
add_dependencies(applewalls ${catkin_EXPORTED_TARGETS})
target_link_libraries(applewalls ${catkin_LIBRARIES})

add_executable(bananawalls src/banana_walls.cpp)
add_dependencies(bananawalls ${catkin_EXPORTED_TARGETS})
target_link_libraries(bananawalls ${catkin_LIBRARIES})

add_executable(housewalls src/house_walls.cpp)
add_dependencies(housewalls ${catkin_EXPORTED_TARGETS})
target_link_libraries(housewalls ${catkin_LIBRARIES})

add_executable(umbrellawalls src/umbrella_walls.cpp)
add_dependencies(umbrellawalls ${catkin_EXPORTED_TARGETS})
target_link_libraries(umbrellawalls ${catkin_LIBRARIES})

add_executable(appledkl src/apple_dkl.cpp)
add_dependencies(appledkl ${catkin_EXPORTED_TARGETS})
target_link_libraries(appledkl ${catkin_LIBRARIES} armadillo ${OpenCV_LIBS} ${OpenMP_LIBS})

add_executable(bananadkl src/banana_dkl.cpp)
add_dependencies(bananadkl ${catkin_EXPORTED_TARGETS})
target_link_libraries(bananadkl ${catkin_LIBRARIES} armadillo ${OpenCV_LIBS} ${OpenMP_LIBS})

add_executable(housedkl src/house_dkl.cpp)
add_dependencies(housedkl ${catkin_EXPORTED_TARGETS})
target_link_libraries(housedkl ${catkin_LIBRARIES} armadillo ${OpenCV_LIBS} ${OpenMP_LIBS})

add_executable(umbrelladkl src/umbrella_dkl.cpp)
add_dependencies(umbrelladkl ${catkin_EXPORTED_TARGETS})
target_link_libraries(umbrelladkl ${catkin_LIBRARIES} armadillo ${OpenCV_LIBS} ${OpenMP_LIBS})

