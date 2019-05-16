#!/usr/bin/python
import math
import rosbag
import rospy
import argparse
import tensorflow as tf
import tf as rostf
import numpy as np
from inserter_detector_py import dataset, convert, plot
from PIL import Image
from queue import Queue
from threading import Thread, currentThread, Lock
import time
import sys

def worker(tid, infile, joint_state_suffix, pc_topic, start, end, outq, writefn, writelock):
  # print "Worker started, reading", start, "to", end
  latest_joint_states = dict()
  sys.stdout.write("&")
  sys.stdout.flush()
  bag = rosbag.Bag(infile, 'r')
  sys.stdout.write("#")
  sys.stdout.flush()
  me = currentThread()
  me.i = 0
  me.tavg = 0
  for topic, msg, t in bag.read_messages(start_time=start, end_time=end):
    t0 = time.time()
    if topic.endswith(joint_state_suffix):
      latest_joint_states[topic] = msg
    elif topic == pc_topic:
      try:
        result = convert.cloud_segments_msg_to_tf_examples(latest_joint_states, msg)
        writelock.acquire()
        for ex in result:
          writefn(ex)
        writelock.release()
      except KeyError:
        pass
    t1 = time.time()
    me.tavg = me.tavg * 0.9 + (t1-t0) * 0.1
    me.i = me.i + 1
  print "worker done"

def _bag_to_tf_record(infile, outfile, joint_state_suffix, pc_topic, tf_topic, num_workers):
  print 'Reading from:', infile
  bag = rosbag.Bag(infile, 'r')
  start = bag.get_start_time()
  end = bag.get_end_time()
  delta = (end-start)/num_workers
  num_messages = bag.get_message_count();
  bag.close()

  print num_messages, 'messages from', start, 'to', end, 'delta', delta

  print 'Writing to:', outfile
  tfWriter = tf.io.TFRecordWriter(outfile)

  count = 0
  latest_joint_states = dict()
  inq = Queue()
  outq = Queue()

  mut = Lock()
  workers = [Thread(target=worker, args=(
    x,
    infile,
    joint_state_suffix,
    pc_topic,
    rospy.Time(start + delta*x),
    rospy.Time(start + delta*(x+1)),
    outq,
    tfWriter.write,
    mut,
  )) for x in xrange(num_workers)]
  # writerT = Thread(target=writer, args=(outq, tfWriter))

  print "Spooling up threads (this may take a couple seconds)"
  for w in workers:
    w.daemon = True
    w.i = 0
    w.tavg = 0
    w.start()
  #writerT.do_run = True
  #writerT.daemon = True
  #writerT.start()

  while any([w.isAlive() for w in workers]):
    time.sleep(1.0)
    print [(w.i, w.tavg) for w in workers]

  [w.join() for w in workers]
  #writerT.do_run = False
  #writerT.join()

  tfWriter.close()
  print 'Finished writing', count, 'examples'

def _verify(filepath):
  print 'Verifying...'

  results = dataset.get_feature_samples([filepath], 64)
  canvas = plot.newCanvas()
  for i, r in enumerate(results):
    plot.pasteImage(canvas, i, r[0].reshape(convert.IMG_SIZE, convert.IMG_SIZE).astype(np.uint8))
    js = convert.prediction_to_joint_state(r[1])
    print js.position
    plot.drawInference(canvas, i, js.position[0], js.position[1], (255, 0, 0))
    plot.drawInference(canvas, i, 0.0, js.position[1], (128, 150, 128))
    plot.drawInference(canvas, i, 0.1, js.position[1], (128, 150, 128))

  canvas = canvas.resize((1024,1024))
  canvas.show()

  print 'Example images shown'

def main():
  parser = argparse.ArgumentParser(description='Converts .bag file to TFRecord.')
  parser.add_argument('--input', type=str, help='input .bag file')
  parser.add_argument('--output', type=str, help='output .tfrecord file')
  parser.add_argument('--verify_only', dest='verify_only', action='store_true', help='do not write; only verify')
  parser.add_argument('--num_workers', dest='num_workers', default=1, type=int, help='number of workers for writing')
  parser.set_defaults(verify_only=False)

  args = parser.parse_args()

  pc_topic = '/inserter/cloud_segments'
  joint_state_suffix = '/joint_states'
  tf_topic = '/tf'

  if args.verify_only:
    print 'Skipping conversion'
  else:
    _bag_to_tf_record(args.input, args.output, joint_state_suffix, pc_topic, tf_topic, args.num_workers)

  _verify(args.output)
