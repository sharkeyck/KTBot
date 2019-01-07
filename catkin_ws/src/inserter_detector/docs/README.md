# Inserter Detector V0.1

## Links

[Robotic Experimentation Plan](https://docs.google.com/document/d/1niBJiZnuH0YFM3ddemYr-tcxUteNDeFz48HjCRtlQMY/view)

## Setup

1. Install tensorflow with GPU support: `sudo pip install tensorflow-gpu`
   *  Also need to install the dependencies for CUDA: https://www.tensorflow.org/install/gpu
      *  Tried cuda 10.0 initially, but the tf library was looking for v9. Follow the "Install CUDA with apt" section to get the right version.
2. Install Keras: `sudo pip install keras`
3. Ensure correct driver version (410.48.0) is installed (was 384.130.0)
4. Install libctpl-dev (for multithreading)
5. Install ros-kinetic-moveit

## Planning

Features:

- Variant 1: SCARA arm (Z-axis plus shoulder, elbow, and gripper)
- Variant 2: Multiple articulated arms
- Adjacent camera for position estimation/localization of single/multiple arms

Milestones:

- (DONE) Hardware design & prototype
- (DONE) Open-loop simulation in gazebo
- (DONE) Rosbag training data extracter
- (DONE) Train DNN for position detection
- (DONE) Closed-loop simulation in gazebo
- Simulation + MoveIt! library using point cloud positioning
  - Including moving arm safely around objects
  - IK-based motion ("go to point")
- Grid of arms, move blocks between pedestals
- Simulate conveyor in gazebo, have arms intercept blocks and
  pass them around on the conveyors (http://gazebosim.org/ariac)
- Physical prototype of conveyor belts
- (DONE) Physical prototype of robotic arms, no depth camera estimation
- Realsense based joint state estimation of single physical robot
- Realsense based joint state estimation of multiple physical robots
- Multiple physical robots passing a physical object between them
- Multiple physical robots passing objects between conveyor belts
- CV based goal planning ("pick up this object and put it there")
- Visual servoing (match offset position with a marker)

Planned, but didn't have to do:

- Install GPU support for PCL (point cloud library) [here](http://pointclouds.org/documentation/tutorials/gpu_install.php) and accelerate point cloud detection stuffs

Applications:

- Shoulder-mounted wearable arm that "chicken-heads" in place. Stabilization routine for holding e.g. drinks
- Can give thumbs-up
- Apparatus for clearing off 3D prints
- Bartending
- Automatic screw sorter
- Electromagnet pickup tool
- Bonus: mount a high speed motor on the end... insta-CNC!
- Control a bunch of robots at once with the same depth camera sensor

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

# Run segmentation (bag --> bag, C++)
rosrun inserter_detector gen_training_data --input src/inserter_detector/data/inserter_with_joint_states.bag --output src/inserter_detector/data/output.herp

# Convert to TFRecord (bag --> TFRecord, Python)
rosrun inserter_detector bag_to_tf_record --input src/inserter_detector/data/inserter_with_joint_states_segmented2.bag --output src/inserter_detector/data/inserter_with_joint_states.tfrecord
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

### Multiple Arms

```
# Simulate 16 robots in a grid
rosrun inserter_detector simulate --num_bots=16

# Issue random commands to all robots
rosrun inserter_detector command_generator --num_bots 16

# Record all robot joint states and the point cloud data to a bag
rosbag record \
  /gazebo/model_states \
  /tf \
  /realsense/depth_registered/points \
  /inserter0/joint_states \
  /inserter1/joint_states \
  /inserter2/joint_states \
  /inserter3/joint_states \
  /inserter4/joint_states \
  /inserter5/joint_states \
  /inserter6/joint_states \
  /inserter7/joint_states \
  /inserter8/joint_states \
  /inserter9/joint_states \
  /inserter10/joint_states \
  /inserter11/joint_states \
  /inserter12/joint_states \
  /inserter13/joint_states \
  /inserter14/joint_states \
  /inserter15/joint_states \

# Run segmentation (bag --> bag, C++)
rosrun inserter_detector gen_training_data --input src/inserter_detector/data/inserter_with_joint_states.bag --output src/inserter_detector/data/output.herp
```

## GDB Cheatsheet

Use GDB to run a node

```
rosrun --prefix 'gdb -ex run --args' inserter_detector gen_training_data --input src/inserter_detector/data/inserter_with_joint_states.bag --output src/inserter_detector/data/inserter_with_joint_states_segmented.bag
```

Use `bt` to get a backtrace

Use `Ctrl+D` to exit
