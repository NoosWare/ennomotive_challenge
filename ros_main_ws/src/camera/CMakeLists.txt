cmake_minimum_required(VERSION 2.8.3)
project(camera)
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(OpenCV REQUIRED)
find_package(raspicam REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options REQUIRED)
catkin_package(
    CATKIN_DEPENDS roscpp
    DEPENDS roscpp
)
add_definitions(-std=c++1y)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES} ${raspicam_CV_LIBS} ${Boost_LIBRARIES})
