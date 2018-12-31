import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import tensorflow as tf

NUM_POINTS = 50
IMG_SIZE = 64
PX_PER_M = 4 * IMG_SIZE

def joint_state_to_prediction(jointstate):
  # jointstate is sensor_msgs/JointState

  rot = jointstate.position[0] #0 - 6.28
  ext = jointstate.position[1] - 0.1, #0.1 - 1.1
  # grip = (jointstate.position[2] - 0.01) / 0.05, # 0.01 -> 0.05

  return [
    math.sin(rot),
    math.cos(rot),
    ext[0],
  ]

def prediction_to_joint_state(prediction):
  # Note: header has no time set
  result = JointState()
  result.header = Header()
  result.name = ['joint1', 'joint2'] #joint5 elided
  result.position = [math.atan2(prediction[0], prediction[1]), prediction[2] + 0.1]
  result.velocity = [0, 0]
  result.effort = [0, 0]
  return result

def flattened_list_to_cloud(flattened_list, frame_id="map"):
  flattened_list.reshape((NUM_POINTS, 3))
  fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
  ]

  header = Header()
  header.frame_id = frame_id
  return pc2.create_cloud(header, fields, flattened_list.reshape((NUM_POINTS, 3)))

def cloud_to_image(cloud):
  def _norm(dim):
    return int(dim * PX_PER_M + IMG_SIZE/2)
  # cloud is sensor_msgs/PointCloud2
  gen = pc2.read_points(cloud, skip_nans=True, field_names=('x', 'y', 'z'))
  data = np.full((IMG_SIZE,IMG_SIZE), 128, dtype=np.uint8)
  coords = map(lambda p: [_norm(p[0]), _norm(p[1]), p[2]], gen)
  zMax = np.max(coords, axis=0)[2]
  zMin = np.min(coords, axis=0)[2]
  for c in coords:
    if c[0] >= IMG_SIZE or c[1] >= IMG_SIZE:
      continue
    # Z value becomes intensity, normalized 0-255
    data[c[0]][c[1]] = int(round((c[2]-zMin) / (zMax - zMin) * 255))
  return data.reshape((IMG_SIZE, IMG_SIZE, 1))

def cloud_to_flattened_list(cloud):
  # cloud is sensor_msgs/PointCloud2
  gen = pc2.read_points(cloud, skip_nans=True, field_names=('x', 'y', 'z'))
  arr = np.array(list(gen))

  # Downsample to exactly 50 points
  fixed = np.random.permutation(arr)[:NUM_POINTS]
  flat = [item for sublist in fixed for item in sublist]
  if len(flat) != NUM_POINTS*3:
    raise Exception('Found example with only ' + len(flat) + ' points, need at least ' + NUM_POINTS)

  return flat

def serial_image_to_2d(serial_img):
  return serial_img.reshape((IMG_SIZE, IMG_SIZE)).astype(np.uint8)

def _joint_feature(jointstate):
  # jointstate is sensor_msgs/JointState
  return tf.train.Feature(float_list=tf.train.FloatList(value=joint_state_to_prediction(jointstate)))

def _flatimg_feature(cloud):
  # cloud is sensor_msgs/PointCloud2
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=[cloud_to_image(cloud).flatten().tobytes()]))

def _cloud_feature(cloud):
  # cloud is sensor_msgs/PointCloud2
  return tf.train.Feature(float_list=tf.train.FloatList(value=cloud_to_flattened_list(cloud)))

def msgs_to_tf_example(joint_state, cloud):
  return tf.train.Example(features=tf.train.Features(feature={
    'joints': _joint_feature(joint_state),
    'cloud_flat': _cloud_feature(cloud),
    'img_flat': _flatimg_feature(cloud),
  }))
