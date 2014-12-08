# Constraint Pouring Controller

## Installation
This package has been developed and tested for Ubuntu 12.04, ROS Hydro, and Gazebo 4, only.

### Prerequisites
Install Gazebo 4 from debians:
  * http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install

Install ROS Hydro:
  * http://wiki.ros.org/hydro/Installation/Ubuntu

Setup a catkin workspace:
  * http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Install various needed packages:
  * ```sudo apt-get install ros-hydro-orocos-kdl ros-hydro-control-toolbox ros-hydro-reflexxes-type2``` ros-hydro-kdl-parser

### Getting the controller plugin from source
Add the needed source repositories to your workspace:
  * ```roscd```
  * ```cd ../src```
  *```wstool set fccl --git -y git@github.com:airballking/fccl.git```
  * ```wstool update fccl```
  * ```wstool set iai_gazebo_pkgs --git -y git@github.com:code-iai/iai_gazebo_pkgs.git```
  * ```wstool update iai_gazebo_pkgs```
  * ```source ~/.bashrc```
  * ```rospack profile```

Build everything using catkin:
  * ```cd ..```
  * ```catkin_make```

### ENVIRONMENT VARIABLES
Finally, use your bashrc to add the lib-subdirectory of your devel-space to the GAZEBO_PLUGIN_PATH, e.g. for my computer:
  ```
  export MY_CATKIN_WS=${HOME}/ros/hydro/catkin_ws/```
  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${MY_CATKIN_WS}/devel/lib
  ```

## Start-up
So far, we only used ROS to build the gazebo plugins. So, to fire up the simulator running the controller plugin, do the following in your shell:
  * ```roscd iai_gazebo_controllers```
  * ```gazebo worlds/constraint_controller_pouring.world --verbose```
