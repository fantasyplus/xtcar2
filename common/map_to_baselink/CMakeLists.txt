cmake_minimum_required(VERSION 2.8.3)
project(map_to_baselink)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nmea_msgs
  roscpp
  tf
  tf2_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  std_msgs
)

catkin_package()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/map_to_baselink.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)
