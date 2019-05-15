# The Easy Way

`sudo docker-compose run (gz|ros1|ros2|tf) bash` for vanilla containers

# The Hard Way

## Starting up RVIZ using docker container and GPU:

Follow instructions at https://github.com/NVIDIA/nvidia-docker to install nvidia-docker

Follow remaining code snippets at http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2
to create a ROS melodic docker image

From https://forums.docker.com/t/start-a-gui-application-as-root-in-a-ubuntu-container/17069

`xhost +local:docker`

Then do `./run.sh`, and `roscore > /dev/null & rosrun rviz rviz`. It works!!!

## Starting gazebo 9 w/ gpu

Use Dockerfile from above but switch image to `FROM gazebo:gzserver9`

Follow deployment example at https://hub.docker.com/_/gazebo

With nvidia-docker, can even start gzclient from within the container!

## Starting ros2 w/ gpu

Same deets, but with `FROM osrf/ros2:nightly`

`ros2 run rviz2 rviz2`

## Starting TF w/ gpu

Same, but with `FROM tensorflow/tensorflow:1.13.1-gpu`

https://www.tensorflow.org/install/docker

## Cross-compilation of ros2 code for ARM/raspi

https://index.ros.org/doc/ros2/Tutorials/Cross-compilation/

# The Plan

Docker containers clustered by purpose
- TF - pose estimation using memory-bound IPC comms for receiving point cloud data
- realsense point cloud reading, https://github.com/IntelRealSense/realsense-ros/issues/386
- ktbot server (mapping, localization, etc nodes)
- ktbot client (odometry, motor control, sound etc nodes)
- moveit/inserter_detector controller

Ideas:
- [depends_on](https://index.ros.org/doc/ros2/Tutorials/Run-2-nodes-in-two-separate-docker-containers/) to indicate deps
- https://micro-ros.github.io/docs/tutorials/getting_started_embedded/ to install on micro for sensor data
- Possible contributions to HRIM? https://acutronicrobotics.com/technology/hrim/
