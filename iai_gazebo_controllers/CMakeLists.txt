cmake_minimum_required(VERSION 2.8.3)
project(iai_gazebo_controllers)

find_package(catkin REQUIRED COMPONENTS giskard)

## Finding system dependencies which come without cmake
find_package(PkgConfig)

#pkg_check_modules(PC_YAMLCPP yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/node.h PATH_SUFFIXES yaml-cpp )
find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp )

if (PKG_CONFIG_FOUND)
  #pkg_check_modules(GAZEBO gazebo)
  find_package(GAZEBO REQUIRED)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

# enable C++11 support
# taken from here: 
# http://answers.ros.org/question/152276/is-there-a-way-to-enable-c11-support-for-catkin-packages/

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
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
