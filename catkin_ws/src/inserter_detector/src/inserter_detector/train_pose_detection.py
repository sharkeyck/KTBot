import argparse
from keras.models import Sequential
from keras.layers import Dense
from inserter_detector.evaluate_input_tensor import EvaluateInputTensor
import tensorflow as tf
from keras import backend as K
import keras

# Parse the record into tensors.
def _parse_function(num_points):
  def result(serialized_example):
    parsed = tf.parse_single_example(serialized_example, features = {
      'label': tf.FixedLenFeature((3), tf.float32),
      'cloud_flat': tf.FixedLenFeature((3*num_points), tf.float32),
    })

    cloud = tf.reshape(parsed['cloud_flat'], (num_points, 3))

    return [cloud, parsed['label']]
  return result

def loadDataset(filenames, num_points=50):
  dataset = tf.data.TFRecordDataset(filenames)
  dataset = dataset.map(_parse_function(num_points))
  return dataset

def layers(x_train_input):
  x = keras.layers.Flatten(input_shape=(-1, 255, 255, 1))(x_train_input)
  x = keras.layers.Dense(1000, activation='relu')(x)
  return x

def main():
  parser = argparse.ArgumentParser(description='Trains a model from an input .tfrecord')
  parser.add_argument('--input', type=str, help='input .tfrecord file')
  args = parser.parse_args()

  sess = K.get_session()

  batch_size = 100
  batch_shape = (batch_size, 50, 3)
  epochs = 5
  num_classes = 10
  test_holdout = 200

  # The capacity variable controls the maximum queue size
  # allowed when prefetching data for training.
  capacity = 10000

  # min_after_dequeue is the minimum number elements in the queue
  # after a dequeue, which ensures sufficient mixing of elements.
  min_after_dequeue = 3000

  # If `enqueue_many` is `False`, `tensors` is assumed to represent a
  # single example.  An input tensor with shape `[x, y, z]` will be output
  # as a tensor with shape `[batch_size, x, y, z]`.
  #
  # If `enqueue_many` is `True`, `tensors` is assumed to represent a
  # batch of examples, where the first dimension is indexed by example,
  # and all members of `tensors` should have the same size in the
  # first dimension.  If an input tensor has shape `[*, x, y, z]`, the
  # output will have shape `[batch_size, x, y, z]`.
  enqueue_many = True

  all_dataset = loadDataset([args.input])
  all_dataset = all_dataset.shuffle(buffer_size=10000)
  all_dataset = all_dataset.repeat()
  data = {"test": all_dataset.take(test_holdout), "train": all_dataset.skip(test_holdout)}

  x_train_batch, y_train_batch = data['train'].shuffle(min_after_dequeue).batch(batch_size)

  model_input = layers.Input(tensor=x_train_batch)
  model_output = layers(model_input)
  train_model = keras.models.Model(inputs=model_input, outputs=model_output)
  train_model.compile(optimizer=keras.optimizers.RMSprop(lr=2e-3, decay=1e-5),
                      loss='categorical_crossentropy',
                      metrics=['accuracy'],
                      target_tensors=[y_train_batch])
  train_model.summary()

  """

  x_test_batch, y_test_batch = tf.train.batch(
      tensors=[data['test'].cloud, data['test'].labels],
      batch_size=batch_size,
      capacity=capacity,
      enqueue_many=enqueue_many,
      num_threads=8)

  # Create a separate test model
  # to perform validation during training
  test_model_input = layers.Input(tensor=x_test_batch)
  test_model_output = layers(test_model_input)
  test_model = keras.models.Model(inputs=test_model_input, outputs=test_model_output)
  test_model.compile(optimizer=keras.optimizers.RMSprop(lr=2e-3, decay=1e-5),
                     loss='categorical_crossentropy',
                     metrics=['accuracy'],
                     target_tensors=[y_test_batch])

  # Fit the model using data from the TFRecord data tensors.
  coord = tf.train.Coordinator()
  threads = tf.train.start_queue_runners(sess, coord)

  train_model.fit(
      epochs=epochs,
      steps_per_epoch=int(np.ceil(data['train'].num_examples / float(batch_size))),
      callbacks=[EvaluateInputTensor(test_model, steps=100)])

  # Save the model weights.
  train_model.save_weights('saved_wt.h5')

  # Clean up the TF session.
  coord.request_stop()
  coord.join(threads)
  K.clear_session()

  # Second Session to test loading trained model without tensors
  x_test = np.reshape(data['test'].cloud, (data['test'].cloud.shape[0], 50, 3))
  y_test = data.test.labels
  x_test_inp = layers.Input(shape=(x_test.shape[1:]))
  test_out = layers(x_test_inp)
  test_model = keras.models.Model(inputs=x_test_inp, outputs=test_out)

  test_model.load_weights('saved_wt.h5')
  test_model.compile(optimizer='rmsprop',
                     loss='categorical_crossentropy',
                     metrics=['accuracy'])
  test_model.summary()

  loss, acc = test_model.evaluate(x_test, y_test, batch_size=batch_size)
  print('\nTest accuracy: {0}'.format(acc))

  """

