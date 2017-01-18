cmake_minimum_required(VERSION 2.8.3)
project(joystick_node)

find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(CATKIN_DEPENDS roscpp)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(joystick_node src/joystick_node.cpp
                             src/joystick.cpp)
target_link_libraries(joystick_node ${catkin_LIBRARIES})
