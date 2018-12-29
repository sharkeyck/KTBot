import argparse
from keras.models import Sequential
from keras.layers import Dense
from inserter_detector.evaluate_input_tensor import EvaluateInputTensor
import tensorflow as tf
from keras import backend as K
import keras
import numpy as np
from tensorflow.python.client import device_lib

sess = tf.Session(config=tf.ConfigProto(log_device_placement=True))


# Parse the record into tensors.
def _parse_function(num_points, img_size):
  def result(serialized_example):
    parsed = tf.parse_single_example(serialized_example, features = {
      'joints': tf.FixedLenFeature((3), tf.float32),
      'cloud_flat': tf.FixedLenFeature((3*num_points), tf.float32),
      'img_flat': tf.FixedLenFeature([], tf.string),
    })

    cloud = tf.reshape(parsed['cloud_flat'], (num_points, 3))
    img = tf.reshape(tf.cast(tf.io.decode_raw(parsed['img_flat'], tf.uint8), tf.float32),(img_size, img_size, 1))

    return [cloud, img, parsed['joints']]
  return result

def loadDataset(filenames, num_points=50, img_size=64):
  dataset = tf.data.TFRecordDataset(filenames)
  dataset = dataset.map(_parse_function(num_points, img_size))
  return dataset

def model_layers(x_train_input):
  # x = keras.layers.Flatten(input_shape=(-1, 50, 3))(x_train_input)
  x = keras.layers.Conv2D(32, kernel_size=(4, 4),
                 activation='relu',
                 padding='same',
                 input_shape=(64, 64, 1))(x_train_input)
  x = keras.layers.AveragePooling2D(pool_size=(4, 4))(x)
  x = keras.layers.Conv2D(64, kernel_size=(4, 4),
                 activation='relu',
                 padding='same')(x)
  x = keras.layers.AveragePooling2D(pool_size=(4, 4))(x)
  x = keras.layers.Conv2D(128, kernel_size=(4, 4),
                 activation='relu',
                 padding='same')(x)
  x = keras.layers.Flatten()(x)
  x = keras.layers.Dense(400, activation='relu')(x)
  x = keras.layers.Dropout(0.25)(x)
  x = keras.layers.Dense(128, activation='linear')(x)
  x = keras.layers.Dropout(0.25)(x)
  x = keras.layers.Dense(3, activation='linear')(x)
  return x

def construct_model(dataset, batch_size):
  # min_after_dequeue is the minimum number elements in the queue
  # after a dequeue, which ensures sufficient mixing of elements.
  min_after_dequeue = 3000

  train_batch = dataset.shuffle(min_after_dequeue).batch(batch_size).repeat()
  train_cloud, train_img, train_joints = train_batch.make_one_shot_iterator().get_next()

  model_input = keras.layers.Input(tensor=train_img)
  model_output = model_layers(model_input)
  train_model = keras.models.Model(inputs=model_input, outputs=model_output)
  train_model.compile(optimizer=keras.optimizers.Adam(), # RMSprop(lr=2e-3, decay=1e-5),
                      loss='mean_squared_error',
                      metrics=['mean_squared_error'],
                      target_tensors=[train_joints])
  return train_model

def do_train(infile):
  sess = K.get_session()

  num_examples = 1700
  batch_size = 50
  epochs = 100
  test_holdout = 200

  all_dataset = loadDataset([infile])
  all_dataset = all_dataset.shuffle(buffer_size=10000)
  data = {"test": all_dataset.take(test_holdout), "train": all_dataset.skip(test_holdout)}

  train_model = construct_model(data["train"], batch_size)
  train_model.summary()

  # Create a separate test model to perform validation during training
  test_model = construct_model(data["test"], batch_size)

  # Fit the model using data from the TFRecord data tensors.
  coord = tf.train.Coordinator()
  threads = tf.train.start_queue_runners(sess, coord)

  tbCallBack = keras.callbacks.TensorBoard(log_dir='./Graph', histogram_freq=0, write_graph=True, write_images=True)

  print "Running..."
  train_model.fit(
      epochs=epochs,
      steps_per_epoch=int(np.ceil(num_examples / float(batch_size))),
      callbacks=[EvaluateInputTensor(test_model, steps=10), tbCallBack])

  print "Saving weights"
  train_model.save_weights('saved_wt.h5')

  print "Cleaning up session"
  coord.request_stop()
  coord.join(threads)
  K.clear_session()


def main():
  parser = argparse.ArgumentParser(description='Trains a model from an input .tfrecord')
  parser.add_argument('--input', type=str, help='input .tfrecord file')
  args = parser.parse_args()

  print "Training, looking for 0.0001 MSE"
  do_train(args.input)

  print "Validating"
  """
  x_test = np.reshape(data['test'].cloud, (data['test'].cloud.shape[0], 50, 3))
  y_test = data.test.labels
  x_test_inp = layers.Input(shape=(x_test.shape[1:]))
  test_out = layers(x_test_inp)
  validation_model = keras.models.Model(inputs=x_test_inp, outputs=test_out)

  validation_model.load_weights('saved_wt.h5')
  validation_model.compile(optimizer='rmsprop',
                     loss='categorical_crossentropy',
                     metrics=['accuracy'])
  validation_model.summary()

  loss, acc = validation_model.evaluate(x_test, y_test, batch_size=batch_size)
  print('\nTest accuracy: {0}'.format(acc))
  """
