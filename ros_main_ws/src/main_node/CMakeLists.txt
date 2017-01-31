cmake_minimum_required(VERSION 2.8.3)
project(main_node)

find_package(catkin REQUIRED COMPONENTS roscpp)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

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
include_directories(${catkin_INCLUDE_DIRS} /usr/include/eigen3)
add_executable(main_node src/main.cpp)
set(LIBRARY_PATH ${LIBRARY_PATH}
                 /lib
                 /usr/lib
                 /usr/local/lib
                 )

# add_dependencies(main_node_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
find_library(OpenANN NAMES openann PATHS LIBRARY_PATH)
message(STATUS "OpeANN found: " ${OpenANN})
if (!OpenANN)
  message(FATAL_ERROR "ERROR: missing OpenANN")
endif(OpenANN)

target_link_libraries(main_node ${catkin_LIBRARIES} 
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${Boost_SERIALIZATION_LIBRARY}
                                ${OpenANN})
