cmake_minimum_required(VERSION 2.8.3)
project(iai_gazebo_controllers)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package()

add_library(constraint_pouring_controller SHARED
  src/iai_gazebo_controllers/constraint_pouring_controller.cc)

target_link_libraries(constraint_pouring_controller
   ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
