# VAUV-simulator

## Description

VAUV-simulator is based on Plankton [open source ROS2 simulator for maritime robotics research] and built to be a platform for developing and testing software for Vortex AUV that will participates in Robosub 2021 competition.

### License

The source code is released under a [ Apache-2.0 License ]

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home)<br />
Maintainer: vortex-co, info@vortex-co.com**

## Table of contents

* [Prerequisites](#Prerequisites)
* [Dependencies](#Dependencies )
* [Installation](#Installation)
* [Launching](#Launching )

## Prerequisites

* ROS2-Eloquent.
* Gazebo9.

## Dependencies

* xacro: ROS2 dashing pkg
* ament_package: Python3 module

## Installation

[1] Installing Gazebo9.

```
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo9
$ sudo apt-get install libgazebo9-dev
$ sudo apt install ros-eloquent-gazebo-ros-pkgs
```

To check for the version of our gazebo:
```
$ gazebo --version
```

Add  " source /usr/share/gazebo/setup.sh" to your .bashrc file.

Extra : Getting the lastes gazebo-ros2  plugins:

Inside ~/vortex_ws/src
```
$ git clone -b eloquent https://github.com/ros-simulation/gazebo_ros_pkgs.git
$ cd ~/vortex_ws
$ colcon build --packages-up-to gazebo_ros_pkgs
```

[2] Installing some dependecies:

Inside ~/vortex_ws/src
```
$ git clone -b dashing-devel https://github.com/ros/xacro.git
$ git clone -b eloquent https://github.com/ros2/launch.git
$ cd ~/vortex_ws
$ colcon build --packages-select xacro
$ colcon build --packages-up-to launch
```
[3] Installing some Pytho3 modules
```
$ sudo apt install ros-eloquent-launch-xml
$ sudo apt install ros-eloquent-launch-yaml
$ cd ~
$ git clone -b eloquent https://github.com/ament/ament_package.git
$ cd ament_package
$ sudo pip install -e .
```
[4] Installing the simulator:
Inside ~/vortex_ws/src
```
$ git clone https://github.com/VorteX-co/VAUV-simulator.git
$ cd ~/vortex_ws
$ colcon build --packages-up-to plankton
```

##Launching

Don’t forget to source workspace/ ROS 2 / Gazebo files if necessary.

[1] Launching a world model:
```
$ros2 launch uuv_gazebo_worlds ocean_waves.launch
```

[2] Spawning an auv model:

This step may requires you to have ros1 melodic installed but not sourced. [ The spawn python script uses roslib module which is a ros1 module]
```
$ros2 launch uuv_descriptions upload_rexrov.launch mode:=default x:=0 y:=0 z:=0 namespace:=rexrov
```

[3] Activating the trajectory tracking controller:

For tele-operatation:
connect a joystick and install the following module '$sudo apt-get install jstest-gtk', then launch the cascaded controller:
```
ros2 launch uuv_control_cascaded_pid joy_velocity.launch uuv_name:=rexrov model_name:=rexrov joy_id:=0
```


For autonomous operation:
```
$ ros2 launch uuv_trajectory_control  rov_pid_controller.launch uuv_name:=rexrov
```

[4] Sending waypoints to follow [for the autonomous operation]:

```
$ ros2 launch uuv_control_utils send_waypoints_file.launch uuv_name:=rexrov
```


