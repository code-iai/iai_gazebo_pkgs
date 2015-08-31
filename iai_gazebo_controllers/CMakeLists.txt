cmake_minimum_required(VERSION 2.8.3)
project(iai_gazebo_controllers)

find_package(catkin REQUIRED COMPONENTS giskard)

## Finding system dependencies which come without cmake
find_package(PkgConfig)

#pkg_check_modules(PC_YAMLCPP yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/node.h PATH_SUFFIXES yaml-cpp )
find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp )

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

## Building plugins using catkin
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})

add_library(GiskardControlPlugin src/iai_gazebo_controllers/giskard_control_plugin)
target_link_libraries(GiskardControlPlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(GiskardVisualizationPlugin src/iai_gazebo_controllers/giskard_visualization_plugin)
target_link_libraries(GiskardVisualizationPlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
