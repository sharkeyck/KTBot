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
  if result.position[0] < 0:
    result.position[0] = result.position[0] + 2*np.pi
  return result

def serial_image_to_2d(serial_img):
  return serial_img.reshape((IMG_SIZE, IMG_SIZE)).astype(np.uint8)

def _joint_feature(jointstate):
  # jointstate is sensor_msgs/JointState
  v = joint_state_to_prediction(jointstate)
  return tf.train.Feature(float_list=tf.train.FloatList(value=v))

def _flatimg_feature(image):
  # cloud is sensor_msgs/PointCloud2
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=[bytes(image.data)]))

def cloud_segments_to_features(joint_states_map, cloud_segments):
  features = []
  for i in xrange(len(cloud_segments.name)):
    inserter_name = cloud_segments.name[i].split('/')[0]
    # position, _ = tfr.lookupTransform('/world', inserter_name + '/base_link', rospy.Time(0))
    joint_state = joint_states_map['/' + inserter_name + '/joint_states']

    # print "===", inserter_name, "==="
    # print "Center\n", position
    # print "Centroid\n", cloud_segments.centroid[i]
    # print "joint_states\n", joint_state

    # print cloud_segments.image[i]

    features.append(tf.train.Example(features=tf.train.Features(feature={
      'joints': _joint_feature(joint_state),
      'img_raw': _flatimg_feature(cloud_segments.image[i]),
    })).SerializeToString())

def raw_img_data_to_shaped_tensor(img_raw):
  return tf.reshape(tf.cast(tf.io.decode_raw(img_raw, tf.uint8), tf.float32),(IMG_SIZE, IMG_SIZE, 1))

def cloud_segments_msg_to_tensor(cloud_segments):
  result = []
  for i in xrange(len(cloud_segments.name)):
    result.append(raw_img_data_to_shaped_tensor(cloud_segments.image[i].data))
  return result
