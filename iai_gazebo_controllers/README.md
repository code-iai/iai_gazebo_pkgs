# Constraint Pouring Controller

## Installation
This package has been developed and tested for Ubuntu 14.04, ROS Indigo, and Gazebo 5, only.

### Install debian dependencies
Install Gazebo 5 from debians:
  * http://gazebosim.org/tutorials?tut=install_ubuntu&ver=5.0&cat=install

Install ROS Indigo:
  * http://wiki.ros.org/indigo/Installation/Ubuntu

Setup a catkin workspace:
  * http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Install dependency KDL from debians:
  * ```sudo apt-get install ros-indigo-orocos-kdl ros-indigo-kdl-parser```

### Download source dependencies
Add dependencies built from source code to your workspace:
  * ```cd /<your-ros-workspace>/src```
  * ```git clone -b catkin git@github.com:airballking/expressiongraph.git```
  * ```git clone git@github.com:airballking/qpOASES.git```
  * ```git clone git@github.com:airballking/giskard.git```

### Download source code of this repository
Add this repository to your workspace:
  * ```git clone git@github.com:code-iai/iai_gazebo_pkgs.git```

### Build everything using catkin
  * ```source ~/.bashrc```
  * ```rospack profile```
  * ```cd ..```
  * ```catkin_make```

### Set up your environment variables
Finally, use your bashrc to add the lib-subdirectory of your devel-space to the GAZEBO_PLUGIN_PATH, e.g. for my computer:
  ```
  export MY_CATKIN_WS=${HOME}/<your-ros-workspace>```
  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${MY_CATKIN_WS}/devel/lib
  ```

## Start-up
### Install custom Gazebo models and plugins
To run the sample worlds, you will need to install two more repositories with Gazebo models and plugins. Please follow the install instructions on them:
  * https://bitbucket.org/zyfang/gz_models
  * https://bitbucket.org/zyfang/sim_cas

### Start simulation
Use a regular call to Gazebo to start the simulation. Note: Select any of the worlds in the sub-directory ```worlds```:
  * ```roscd iai_gazebo_controllers```
  * ```gazebo worlds/constraint_controller_pouring_01.world --verbose -u```
