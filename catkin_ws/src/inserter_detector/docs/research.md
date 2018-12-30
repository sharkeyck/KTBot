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

- Looks like the joint state information isn't actually the current joint state - it's the commanded joint state :(
  - This means our training data is bad!
- Try scaling Z axis image intensity (0-255 for each input)
- Or use int8 instead of uint8 for intensity values
- Try showing a grid of images - ones that perform well, vs ones that don't, as an output of test_pose_detection
- Use matplotlib (https://realpython.com/python-matplotlib-guide/) to show a histogram of "good" detections vs "bad" detections in test.