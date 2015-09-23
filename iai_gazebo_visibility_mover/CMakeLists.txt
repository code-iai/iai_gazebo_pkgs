cmake_minimum_required(VERSION 2.8.3)
project(iai_gazebo_visibility_mover)

find_package(catkin REQUIRED roscpp gazebo_msgs sensor_msgs cv_bridge image_transport)

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES iai_gazebo_visibility_mover)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}/visibility_mover.cpp
  src/${PROJECT_NAME}/visibility_mover_node.cpp)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(iai_pixel_counter
  src/${PROJECT_NAME}/pixel_counter.cpp
  src/${PROJECT_NAME}/pixel_counter_node.cpp)
add_dependencies(iai_pixel_counter
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(iai_pixel_counter ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
