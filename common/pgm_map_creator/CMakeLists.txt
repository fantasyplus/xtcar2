cmake_minimum_required(VERSION 2.8.8)
project(pgm_map_creator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set (CMAKE_CXX_FLAGS "-O2 -Wall -std=c++11 ${CMAKE_CXX_FLAGS}")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pgm_map_creator
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)



link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

# ${Boost_LIBRARIES}
add_executable (request_publisher src/request_publisher.cc)
target_link_libraries(request_publisher collision_map_creator_msgs  ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(request_publisher collision_map_creator_msgs)

add_library(collision_map_creator SHARED src/collision_map_creator.cc )
target_link_libraries(collision_map_creator collision_map_creator_msgs ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(collision_map_creator collision_map_creator_msgs)

