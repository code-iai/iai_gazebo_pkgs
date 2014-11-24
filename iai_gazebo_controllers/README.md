# Constraint Pouring Controller

## Installation
Use the regular catkin-way to build this.

Afterwards, use your bashrc to add the lib-subdirectory of your devel-space to the GAZEBO_PLUGIN_PATH, e.g. for my computer:
```export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/georg/ros/hydro/catkin_ws/devel/lib```

## Start-up
So far, we only use ROS to build the gazebo plugins. So, to fire up the simulator, do the following in your shell:
```roscd iai_gazebo_controllers```
```gazebo worlds/constraint_controller_pouring.world --verbose```
