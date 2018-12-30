#!/usr/bin/python
import math
import rosbag
import argparse
import tensorflow as tf
import numpy as np
from inserter_detector import dataset, convert
from PIL import Image

def _bag_to_tf_record(infile, outfile, joint_state_topic, pc_topic):
  topics = [pc_topic, joint_state_topic]
  print 'Reading from:', infile
  bag = rosbag.Bag(infile)
  num_messages = bag.get_message_count(topic_filters=topics);
  print num_messages, 'messages'

  print 'Writing to:', outfile
  writer = tf.io.TFRecordWriter(outfile)

  latest_joint_state = None

  i = 0
  count = 0
  for topic, msg, t in bag.read_messages(topics=topics):
    i += 1
    if not i % 1000:
      print 'Export: {}/{}'.format(i, num_messages)

    if topic == joint_state_topic:
      latest_joint_state = msg
      continue

    if topic != pc_topic:
      raise Exception('Unknown message on topic ', topic)

    example = convert.msgs_to_tf_example(latest_joint_state, msg)
    count += 1
    writer.write(example.SerializeToString())

  writer.close()
  bag.close()
  print 'Finished writing', count, 'examples'

def _verify(filepath):
  print 'Verifying...'

  v = dataset.get_feature_samples([filepath], 1)[0]

  print 'Example:', v
  img = Image.fromarray(convert.serial_image_to_2d(v[1]), 'L')
  img = img.resize((800, 800)) # Resize so it can be easily viewed
  img.show()
  print 'Example image shown'

def main():
  parser = argparse.ArgumentParser(description='Converts .bag file to TFRecord.')
  parser.add_argument('--input', type=str, help='input .bag file')
  parser.add_argument('--output', type=str, help='output .tfrecord file')
  parser.add_argument('--verify_only', dest='verify_only', action='store_true', help='do not write; only verify')
  parser.set_defaults(verify_only=False)

  args = parser.parse_args()

  pc_topic = '/realsense/depth_registered/points'
  joint_state_topic = '/inserter/joint_states'

  if args.verify_only:
    print 'Skipping conversion'
  else:
    _bag_to_tf_record(args.input, args.output, joint_state_topic, pc_topic)

  _verify(args.output)
