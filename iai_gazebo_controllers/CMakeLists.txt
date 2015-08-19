cmake_minimum_required(VERSION 2.8.3)
project(iai_gazebo_controllers)

find_package(catkin REQUIRED COMPONENTS)

## Finding system dependencies which come with cmake
#find_package(Boost REQUIRED COMPONENTS system)

## Finding system dependencies which come without cmake
find_package(PkgConfig)

#pkg_check_modules(PC_YAMLCPP yaml-cpp)
#find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/node.h PATH_SUFFIXES yaml-cpp )
#find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp )

pkg_check_modules(GAZEBO gazebo)

## Building plugins using catkin
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
#  ${YAML_CPP_INCLUDE_DIRS})

link_directories(
  ${GAZEBO_LIBRARY_DIRS})

#add_library(ConstraintControlPlugin
#  src/iai_gazebo_controllers/constraint_control_plugin.cc)
#target_link_libraries(ConstraintControlPlugin
#  ${GAZEBO_LIBRARIES} 
#  ${Boost_LIBRARIES} 
#  ${catkin_LIBRARIES})
