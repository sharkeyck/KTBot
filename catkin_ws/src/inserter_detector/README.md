# Inserter Detector V0.1

## Links

[Robotic Experimentation Plan](https://docs.google.com/document/d/1niBJiZnuH0YFM3ddemYr-tcxUteNDeFz48HjCRtlQMY/view)

## Setup

TODO

## Planning

Features:

- Variant 1: SCARA arm (Z-axis plus shoulder, elbow, and gripper)
- Variant 2: Multiple articulated arms
- Adjacent camera for position estimation/localization of single/multiple arms

Milestones:

- Hardware design & prototype (DONE)
- Open-loop simulation in gazebo (DONE)
- Rosbag training data extracter
- Train DNN for position detection
- Closed-loop simulation in gazebo
- Simulation + MoveIt! library
- Install GPU support for PCL (point cloud library) [here](http://pointclouds.org/documentation/tutorials/gpu_install.php) and accelerate point cloud detection stuffs
- Working prototype
- Move around safely (with limits)
- IK-based motion ("go to point")
- CV goal planning ("pick up this object and put it there")
- Visual servoing (match offset position with a marker)

Applications:

- Shoulder-mounted wearable arm that "chicken-heads" in place. Stabilization routine for holding e.g. drinks
- Can give thumbs-up
- Apparatus for clearing off 3D prints
- Bartending
- Automatic screw sorter
- Electromagnet pickup tool
- Bonus: mount a high speed motor on the end... insta-CNC!


## Examples

**Gazebo Simulation**

```
roslaunch inserter_detector gazebo.launch
```

**RViz Visualization**

```
roslaunch inserter_detector inserter_rviz.launch
```

**Pose Detection**

Testing pose detection from a recorded bag of point cloud data

```
roslaunch inserter_detector test_pose_detection.launch
```


