cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar_planner_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

find_package(
  catkin REQUIRED 
  COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    tf2
    tf2_ros
    visualization_msgs
    pluginlib
    costmap_2d
    nav_core
    #insert by xt
    high_performence_hybrid_astar_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hybrid_astar_planner_ros
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    tf2
    tf2_ros
    visualization_msgs
    pluginlib
    costmap_2d
    nav_core
    #insert by xt
    high_performence_hybrid_astar_ros
)

add_library(${PROJECT_NAME}
  src/hybrid_astar_planner_ros.cpp
)

add_dependencies(${PROJECT_NAME}
  {${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES hys_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

