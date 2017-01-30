cmake_minimum_required(VERSION 2.8.3)
project(imu)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)
add_definitions(-std=c++1y)

add_executable(imu src/imu_node.cpp
                   src/I2CBus.cpp
                   src/L3G.cpp
                   src/LSM303.cpp
                   src/MinIMU9.cpp
                   src/imu_broadcaster.cpp
                   src/imu_node.cpp
)

target_link_libraries(imu ${catkin_LIBRARIES})
