cmake_minimum_required(VERSION 2.8.3)
project(mrpt_slam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(MRPT REQUIRED  base
                            obs
                            slam
                            maps
                            hwdrivers
)

# Rplidar drivers
find_library(RPLIDAR_LIBRARY NAMES rplidar_sdk HINTS /usr/local/lib)

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
#  LIBRARIES mrpt_slam
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_definitions(-std=c++1y)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
                      ${catkin_INCLUDE_DIRS}
                   )
## Declare a C++ executable
# add_executable(mrpt_slam_node src/slam_node.cpp
#                               src/imu/I2CBus.cpp
#                               src/imu/L3G.cpp
#                               src/imu/LSM303.cpp
#                               src/imu/MinIMU9.cpp
#                               src/imu/MinIMU9.cpp
#                               src/imu/imu_broadcaster.cpp
#                               src/lidar.cpp
#               )

 add_executable(mrpt_slam_node src/slam_node.cpp
                               src/imu/I2CBus.cpp
                               src/imu/L3G.cpp
                               src/imu/LSM303.cpp
                               src/imu/MinIMU9.cpp
                               src/imu/MinIMU9.cpp
                               src/imu/imu_broadcaster.cpp
               )


## Specify libraries to link a library or executable target against
 target_link_libraries(mrpt_slam_node ${catkin_LIBRARIES}
                                      ${MRPT_LIBS}
                                      ${RPLIDAR_LIBRARY}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mrpt_slam mrpt_slam_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
