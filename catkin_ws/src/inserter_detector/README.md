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

**Make just this package**

```
catkin_make --pkg inserter_detector
```

**Gazebo Simulation**

```
roslaunch inserter_detector gazebo.launch
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

**Get joint states and point cloud data for training**

```
# Start up gazebo sim as usual, then run this command to capture data. Press Ctrl+C to stop capture
rosbag record /realsense/depth_registered/points /inserter/joint_states --output_name=capture.bag

# Convert the bag into training data
rosrun inserter_detector gen_training_data src/inserter_detector/data/inserter_with_joint_states.bag src/inserter_detector/data/output.herp
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

## GDB Cheatsheet

Use GDB to run a node

```
rosrun --prefix 'gdb -ex run --args' inserter_detector gen_training_data src/inserter_detector/data/inserter_with_joint_states.bag src/inserter_detector/data/inserter_with_joint_states_segmented.bag
```

Use `bt` to get a backtrace

Use `Ctrl+D` to exit
