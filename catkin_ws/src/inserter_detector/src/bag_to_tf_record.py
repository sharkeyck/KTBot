#!/usr/bin/python

import rosbag
import sys
import argparse
import tensorflow as tf
import numpy as np
import sensor_msgs.point_cloud2 as pc2

NUM_POINTS = 50

parser = argparse.ArgumentParser(description='Converts .bag file to TFRecord.')
parser.add_argument('--input', type=str, help='input .bag file')
parser.add_argument('--output', type=str, help='output .tfrecord file')
parser.add_argument('--verify_only', dest='verify_only', action='store_true', help='do not write; only verify')
parser.set_defaults(verify_only=False)

args = parser.parse_args()

pc_topic = "/realsense/depth_registered/points"
joint_states_topic = "/inserter/joint_states"
topics = [pc_topic, joint_states_topic]

def convert():
  print "Reading from:", args.input
  bag = rosbag.Bag(args.input)
  num_messages = bag.get_message_count(topic_filters=topics);
  print num_messages, "messages"

  print "Writing to:", args.output
  writer = tf.io.TFRecordWriter(args.output)

  def _int64list_feature(values):
    return

  def _joint_feature(jointstate):
    # jointstate is sensor_msgs/JointState
    return tf.train.Feature(float_list=tf.train.FloatList(value=jointstate.position))

  def _cloud_feature(cloud):
    # cloud is sensor_msgs/PointCloud2
    gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
    arr = np.array(list(gen))

    # Downsample to exactly 50 points
    fixed = np.random.permutation(arr)[:NUM_POINTS]
    flat = [item for sublist in fixed for item in sublist]
    if len(flat) != NUM_POINTS*3:
      print "HERP", len(flat)

    return tf.train.Feature(float_list=tf.train.FloatList(value=flat))

  latest_joint_state = None

  i = 0
  for topic, msg, t in bag.read_messages(topics=topics):
      i += 1
      if not i % 1000:
        print 'Export: {}/{}'.format(i, num_messages)

      if topic == joint_states_topic:
        latest_joint_state = msg
        continue

      if topic != pc_topic:
        raise Exception("Unknown message on topic ", topic)

      example = tf.train.Example(features=tf.train.Features(feature={
        'label': _joint_feature(latest_joint_state),
        'cloud_flat': _cloud_feature(msg),
      }))

      writer.write(example.SerializeToString())

  print 'Closing'

  writer.close()
  bag.close()

  print 'Finished writing'

if args.verify_only:
  print "Skipping conversion"
else:
  convert()

print "Verifying..."

dataset = tf.data.TFRecordDataset([args.output])

# Parse the record into tensors.
def _parse_function(serialized_example):
  parsed = tf.parse_single_example(serialized_example, features = {
    'label': tf.FixedLenFeature((3), tf.float32),
    'cloud_flat': tf.FixedLenFeature((3*NUM_POINTS), tf.float32),
  })

  image = tf.reshape(parsed['cloud_flat'], (NUM_POINTS, 3))

  return parsed['label'], image

dataset = dataset.map(_parse_function)

# Create a one-shot iterator
iterator = dataset.make_one_shot_iterator()

# Get example value
print "Example:"
next_element = iterator.get_next()
with tf.Session() as sess:
  print sess.run(next_element)
