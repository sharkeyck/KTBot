# KTBot

## Setup

1. Install Docker
2. Build container: `docker build -t my-ros-app .`
3. Install XMing Windows System Server (Google for download link for your OS)
4. Create the container (Make sure to change the IP address and username folder to your computer's / user's): `docker run -it --rm --name rostest -e DISPLAY=YOURIP:0.0 -v C:\Users\USERNAME\Documents\KTBot\catkin_ws:/root/catkin_ws my-ros-app:latest`

## Running the Robot

TODO: how to use XMing? How to start / run the code?

`roslaunch ktbot stdr_server_ktbot.launch`

## ROS Details / References

Laser message format: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
Setup reference: https://github.com/baudren/NoteOrganiser/wiki/Docker,-QML,-XServer-on-Windows
ROS basics reference: http://wiki.ros.org/turtlesim

STDR simulator launcher

## Next steps

- Localization, motion planning, and mapping
  - Research `navigation` and `gmapping` libraries, construct settings files for both of 'em.
  - Using teleop, construct a map
  - Point-and-click SLAM navigation
- Gazebo
  - Complete "model structure & requirements", "make a mobile robot", and "add a sensor to a robot" in [Build a robot](http://gazebosim.org/tutorials?cat=build_robot) tutorial set.
  - Create a "Neato" lidar module and publish it to osrf/gazebo_models
  - Adapt a model plugin that retrieves the gazebo simulated lidar data and publishes it on a ros topic. (see [gazebo_ros_pkgs tutorials](http://gazebosim.org/tutorials?cat=connect_ros))
  - Create a servo simulator model plugin (LX-16A) that parses/interprets serial data and affects a rotating joint. Consider upstreaming it in gazebo_ros_pkgs.
  - Create a KTBot-like robot with rotatable wheels and top lidar
  - Construct a .world file to launch the ktbot and register the lidar/servo plugins
  - Try a full integration test, running the simulated bot using [teleop via ROS](http://wiki.ros.org/stdr_simulator/Tutorials/Using%20turtlebot%20for%20teleoperation)
  - Construct hackerhouse model using [this tutorial](http://gazebosim.org/tutorials?cat=build_world&tut=building_editor) and drive the KTbot around in it.
- Autonomy
  - Build charge station (switch-based wireless charger)
  - Setup low battery sensing (via servos?)
  - Return-to-base behavior when low on battery or objective complete
  - Think up fun objectives!

## Packages

- gmapping
- navigation
- stdr_simulator
- neato_robot (https://github.com/mikeferguson/neato_robot)

## Python (pip) dependencies

- inject
- paho-mqtt


## Gazebo Woes

// `docker run -it -p 7681:7681 -p 8080:8080 --name gzweb giodegas/gzweb /bin/bash`

- Use ubuntu 16.04.1 (ver \*.3 uses nouveau drivers, which with my nvidia card doesn't work out at all.)
- Install ros-kinetic-desktop-full
- Install gazebo7 as the ros-gazebo package requires it
- URDF spec needed for running a ros robot
