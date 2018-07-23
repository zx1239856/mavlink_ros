cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_definitions(-std=c++11)

add_executable(keyboard_control src/keyboard_control.cpp src/ky_control.cpp)
target_link_libraries(keyboard_control ${catkin_LIBRARIES})
