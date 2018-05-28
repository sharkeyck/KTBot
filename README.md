# KTBot


## Running the Robot

`roslaunch ktbot stdr_server_ktbot.launch`

## ROS Details / References

Laser message format: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
Setup reference: https://github.com/baudren/NoteOrganiser/wiki/Docker,-QML,-XServer-on-Windows
ROS basics reference: http://wiki.ros.org/turtlesim

STDR simulator launcher

## Next steps

- Fix gazebo model for new base size
- Publish correct transforms such as base_laser_link
- get RViz working!
- GMAPPING AND MOOOOOOORE!

- Mechanical fixes
  - do auto port recognition
  - More stable wheel base

- Software
  -
- Driver fixes
  - Fix servo settings for drifting servo
  - PWM control of wheel speed in servo
- Gazebo
  - Create a servo simulator model plugin (LX-16A) that parses/interprets serial data and affects a rotating joint. Consider upstreaming it in gazebo_ros_pkgs.
  - Construct a .world file to launch the ktbot and register the lidar/servo plugins
  - Construct hackerhouse model using [this tutorial](http://gazebosim.org/tutorials?cat=build_world&tut=building_editor) and drive the KTbot around in it.
- Autonomy
  - Build charge station (switch-based wireless charger)
  - Setup low battery sensing (via servos?)
  - Return-to-base behavior when low on battery or objective complete
  - Think up fun objectives!

For V2
- Fix bot size & cable routing/management
- Gear wheels for higher speed
- ARBS

## Packages

- gmapping
- navigation
- stdr_simulator
- neato_robot (https://github.com/mikeferguson/neato_robot)

## Python (pip) dependencies

- inject
- paho-mqtt
- msgpack
- pyserial

## Gazebo Woes

- Use ubuntu 16.04.1 (ver \*.3 uses nouveau drivers, which with my nvidia card doesn't work out at all.)
- Install ros-kinetic-desktop-full
- Install gazebo7 as the ros-gazebo package requires it
- URDF spec needed for running a ros robot

## Odroid
- Installed Ubuntu 16.04, added wifi to network interfaces, no wpa-config
- Adding zsh, git, vim
- installed ros-kinetic-ros-base
- added KTBot repo, missing the followig dependencies:
  - ros-kinetic-ros-control-boilerplate
  - ros-kinetic-joint-state-controller
  - ros-kinetic-robot-state-publisher
  - ros-kinetic-diff-drive-controller
  - davetcoleman/ros-control-boilerplate repo from Github
  - davetcoleman/gflags from Github NOPE, use: libgflags-dev
  - ros-kinetic-mqtt-bridge
  - python-pip: paho-mqtt, inject, msgpack, pyserial
  - ros-kinetic-ros-control, ros-kinetic-ros-controllers
  - ssh
  - ros-kinetic-xacro
- echo catkin_ws/devel/setup.zsh to zshrc
-changed hostname from odroid to ktbot- login as kt, /etc/hosts and /etc/hostname 
-installed avahi daemon for mDNS things

 
### Debugging servos

1. roscd lx16a_driver && cd src
2. python example.py /dev/ttyUSB0 1
   3. Servo ids are 0 or 1

## Failed At:

* Installing Qemu for ROS Compilation that Doesn't Suck
  *  https://www.cnx-software.com/2012/03/08/how-to-build-qemu-system-arm-in-linux/
* Docker gazebo
  1. Install Docker
  2. Build container: `docker build -t my-ros-app .`
  3. Install XMing Windows System Server (Google for download link for your OS)
  4. Create the container (Make sure to change the IP address and username folder to your computer's / user's): `docker run -it --rm --name rostest -e DISPLAY=YOURIP:0.0 -v C:\Users\USERNAME\Documents\KTBot\catkin_ws:/root/catkin_ws my-ros-app:latest`
  *  `docker run -it -p 7681:7681 -p 8080:8080 --name gzweb giodegas/gzweb /bin/bash`
