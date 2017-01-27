cmake_minimum_required(VERSION 2.8.3)
project(main_node)

find_package(catkin REQUIRED COMPONENTS roscpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
#  LIBRARIES main_node
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-std=c++1y ${CMAKE_CXX_FLAGS}")
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(main_node src/main.cpp)

# add_dependencies(main_node_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_node ${catkin_LIBRARIES})
