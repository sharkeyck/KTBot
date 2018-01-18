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

- Submit PR For parsing stdr_parser xml stuff... in stdr_xml_parser.cpp

```
Node* new_node = new Node();
new_node->file_origin = n->file_origin;
TiXmlNode* pChild;
```

- Simulate lidar (stdr simulator)
- Rip stuff out of turtlebot_navigation (http://wiki.ros.org/turtlebot_navigation)
- Auto-detect serial port based on device ID
- Configure robot from (../resources/robots/pandora_robot.yaml)
- Copy server_with_map_and_gui_plus_robot.launch from stdr_launchers, customize
- Setup teleop with http://wiki.ros.org/stdr_simulator/Tutorials/Using%20turtlebot%20for%20teleoperation

## Packages

- gmapping
- navigation
- stdr_simulator
- neato_robot (https://github.com/mikeferguson/neato_robot)

## Python (pip) dependencies

- inject
- paho-mqtt
