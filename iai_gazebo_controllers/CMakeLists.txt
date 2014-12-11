cmake_minimum_required(VERSION 2.8.3)
project(iai_gazebo_controllers)

#find_package(catkin REQUIRED COMPONENTS fccl_core fccl_conversions)

## Finding system dependencies which come with cmake
find_package(Boost REQUIRED COMPONENTS system)

## Finding system dependencies which come without cmake
find_package(PkgConfig)
#pkg_check_modules(PC_YAMLCPP yaml-cpp)
#find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/node.h PATH_SUFFIXES yaml-cpp )
#find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp )

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

# needed if gazebo is installed from source
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})
  #${YAML_CPP_INCLUDE_DIRS})

add_library(constraint_pouring_controller SHARED
  src/iai_gazebo_controllers/constraint_pouring_controller.cc)
target_link_libraries(constraint_pouring_controller
   ${GAZEBO_LIBRARIES} 
   ${Boost_LIBRARIES} 
   ${catkin_LIBRARIES} 
   ${YAML_CPP_LIBRARIES})

   
add_library(WorldControlPlugin SHARED
  src/iai_gazebo_controllers/WorldControlPlugin.cc)
target_link_libraries(WorldControlPlugin
   ${GAZEBO_LIBRARIES} 
   ${Boost_LIBRARIES} 
   ${catkin_LIBRARIES})