cmake_minimum_required(VERSION 2.8.3)
project(joystick_node)

find_package(catkin REQUIRED COMPONENTS roscpp)

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
catkin_package(CATKIN_DEPENDS roscpp DEPENDS system_lib)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(joystick_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joystick_node src/joystick_node.cpp
                             src/joystick.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(joystick_node ${catkin_LIBRARIES})
