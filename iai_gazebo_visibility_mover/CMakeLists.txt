cmake_minimum_required(VERSION 2.8.3)
project(iai_gazebo_visibility_mover)

find_package(catkin REQUIRED roscpp gazebo_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES iai_gazebo_visibility_mover)

include_directories(include)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}/visibility_mover.cpp
  src/${PROJECT_NAME}/visibility_mover_node.cpp)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
