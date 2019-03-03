# Research Notes

Spin Image: https://pdfs.semanticscholar.org/30c3/e410f689516983efcd780b9bea02531c387d.pdf

- Not nearly as simple as just looking at the image from Z axis and converting Z to image intensity. Trying Z projection first.

Output angle

- 0-1 scaling of rotation has problems at edges
- Could try splitting into (cos(theta), sin(theta)) for continuous prediction space
  - https://stats.stackexchange.com/questions/218407/encoding-angle-data-for-neural-network
  - Observed a pretty significant improvement in loss from this change

2D convolution and downsampling

- Observed massive improvement in loss when shifting from downsampling (4,4) to downsampling (2,2).

Running in C++

- Maybe just make a python node for now.
- May need to use http://wiki.ros.org/tensorflow_ros_cpp if want to run a C++ model.

GPU utilization check

- Can use `nvidia-smi` console command while training/inference is running. If there's a process named "python", it's using the GPU!


## Ideas

- (DONE) Looks like the joint state information isn't actually the current joint state
  - error was due to bug in display code!
- (DONE) Try scaling Z axis image intensity (0-255 for each input) or use int8 instead of uint8 for intensity values
- (DONE) Try showing a grid of images - ones that perform well, vs ones that don't, as an output of test_pose_detection
- (NAH) Use matplotlib (https://realpython.com/python-matplotlib-guide/) to show a histogram of "good" detections vs "bad" detections in test.
- (DONE) Angle error (1.2-1.3deg) may be a factor of image resolution. What if we increased the image size?

### Possible performance improvements

- Use rosbag_direct_write with custom CloudSegments serializer (https://github.com/osrf/rosbag_direct_write/blob/a6ea09be0b307866f090929de7b13332db09c3d9/test/direct_PointCloud2.h)
- For performance, could bundle the inserter segmentation and inference into a single nodelet
  - This is a C++ only construct, so would have to learn how to run TF models in C++.

### MoveIt Notes

MoveIt seems to make a lot of assumptions that robots have 6DOF end effectors, resulting in e.g. not being able to remove the "orientation constraint" when planning a Pick() operation
via the MoveGroup API. This will probably require a patch.

Some discussion here: https://groups.google.com/forum/#!msg/moveit-users/-Eie-wLDbu0/IEx4w7hRmboJ

Likely will need to create a custom ConstraintSamplerAllocator to ignore orientation in a configurable way.
The tutorial (http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/custom_constraint_samplers/custom_constraint_samplers_tutorial.html)
seems indicates that a custom constraint sampler can be loaded instead of the default (https://sourcegraph.com/github.com/ros-planning/moveit@d70d44fea39f41bc64b7687dd059914480a4308e/-/blob/moveit_core/constraint_samplers/src/default_constraint_samplers.cpp)

Updates 2019-01-17:
- Looking further at the constraint sampler it appears to not be for goal constraints, only movement constraints. So that's not what we want.
- Tried increasing the DoF of the robot, but it started feeling like a stack of hacks that are only really necessary to satisfy the pick & place API
- Suggest next attempt to be "manual" pick and place, using manually-defined move goals with no orientation constraints.
  - For pick
    - "move close" to start, retracted from destination by some amount
    - "move to intercept" to get in position and clamp it
    - "move up" to pick it up
  - For place
    - "move close" to start, get to pre-approach position
    - "move to intercept" with drop position and drop it
    - "retract" to get out of the way.

Updates 2019-01-18:
- Last night, got the robot planning and executing using the "manual" pick and place idea from the 17th.
- This morning, started researching how to get multiple arms planned together. Naive approach is to have a bunch of instances of
  moveit controlling each arm, but that could result in crashing between arms and seems like a bit of a waste.
  - Apparently it's possible (as of 2014) to supply a move group containing multiple arms (see http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/move_group_interface_tutorial.html#dual-arm-pose-goals) and plan/execute on both of them within the same group.
  - Nested groups are apparently allowed like so: https://github.com/kth-ros-pkg/pr2_ft_moveit_config/blob/hydro/config/pr2.srdf
  - Will need to dynamically generate the inserter.srdf file and basically all the moveit yaml files. Looks like I'm writing another simulation.py file.
  - Discussion: https://groups.google.com/forum/#!topic/moveit-users/vV5GR2kFR9o
- Suggestion from craig: do continuous planning with adjustable targets
  - "trajectory replacement" is the term typically used here.
  - This would allow for perceived-independent action for each arm  while also preventing collisions among arms.


### References

- https://www.cs.colorado.edu/~mozer/Teaching/Computational%20Modeling%20Prelim/Otte.pdf Applications of machine learning to path planning. Use of ML to provide a better sensor than the depth camera is considered here as a "meta-sensor". Specifically section 3.3 ("learning the representation").
