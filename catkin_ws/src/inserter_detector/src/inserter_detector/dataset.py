import tensorflow as tf
import convert

def _parse_function(serialized_example):
  # Parse the record into tensors.
  parsed = tf.parse_single_example(serialized_example, features = {
    'joints': tf.FixedLenFeature((3), tf.float32),
    'cloud_flat': tf.FixedLenFeature((3*convert.NUM_POINTS), tf.float32),
    'img_flat': tf.FixedLenFeature([], tf.string),
  })

  cloud = tf.reshape(parsed['cloud_flat'], (convert.NUM_POINTS, 3))
  img = tf.reshape(tf.cast(tf.io.decode_raw(parsed['img_flat'], tf.uint8), tf.float32),(convert.IMG_SIZE, convert.IMG_SIZE, 1))

  return [cloud, img, parsed['joints']]

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
