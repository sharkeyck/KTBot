import tensorflow as tf
from inserter_detector_py import convert

def _parse_function(serialized_example):
  # Parse the record into tensors.
  parsed = tf.parse_single_example(serialized_example, features = {
    'joints': tf.FixedLenFeature((3), tf.float32),
    'img_raw': tf.FixedLenFeature([], tf.string),
  })

  img = convert.raw_img_data_to_shaped_tensor(parsed['img_raw'])
  # img = parsed['img_raw']

  return [img, parsed['joints']]

def load_features(filenames):
  # Load a dataset converted with bag_to_tf_record.
  dataset = tf.data.TFRecordDataset(filenames)
  dataset = dataset.map(_parse_function)
  return dataset

def get_feature_samples(filenames, num_samples=1):
  features = load_features(filenames).shuffle(10000)

  # Create a one-shot iterator
  iterator = features.make_one_shot_iterator()

  # Get example value(s)
  results = []
  next_element = iterator.get_next()
  with tf.Session() as sess:
    for i in xrange(num_samples):
      v = sess.run(next_element)
      results.append(v)
  return results
