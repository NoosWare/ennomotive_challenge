cmake_minimum_required(VERSION 2.8.3)
project(imu_node)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
include_directories(/usr/include/eigen3)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_node
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(imu_node src/imu_node.cpp
                        src/I2CBus.cpp
                        src/L3G.cpp
                        src/LSM303.cpp
                        src/MinIMU9.cpp
                        src/MinIMU9.cpp
              )

# add_dependencies(imu_node_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(imu_node ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS imu_node imu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
