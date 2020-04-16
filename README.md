# Simple Quadrotor UAV PID Control

This repository include an example of using ROS and Gazebo to position control a quadrotor UAV, using C++.
A simple PID controller (not the internal ROS controller) is used as the controller.

## Initial Setup 
```sh
git clone https://github.com/fdcl-gwu/gazebo_uav_control.git
cd gazebo_uav_control
git submodule update --init
catkin_make
cd devel && source setup.bash && cd ../
```

## Running the code
```sh
roslaunch uav_gazebo simple_world.launch 
```

## Explanation
This code is organized as follows:
* `src/uav_control`: plugin for controlling the UAV
* `src/uav_gazebo`: Gazebo simulation related files

This uses `uav_control/src/control_plugin.cpp` file which uses the Gazebo model plugin as described [here](http://gazebosim.org/tutorials?tut=ros_gzplugins).
Determining the current state and control input is done inside this file.

The states of the UAV can be checked with
```
rostopic echo /states
```
after launching Gazebo simulation.
