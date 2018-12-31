import argparse
from keras.models import Sequential
from keras.layers import Dense
from inserter_detector.evaluate_input_tensor import EvaluateInputTensor
import tensorflow as tf
from keras import backend as K
import keras
import numpy as np
from inserter_detector import dataset
from tensorflow.python.client import device_lib
from inserter_detector.model import model_layers

sess = tf.Session(config=tf.ConfigProto(log_device_placement=True))

NUM_EXAMPLES = 1700
BATCH_SIZE = 100
EPOCHS = 100
TEST_HOLDOUT = 200
BUFFER_SIZE = 10000

def _construct_model(data):
  train_batch = data.batch(BATCH_SIZE).repeat()
  _, img, joints = train_batch.make_one_shot_iterator().get_next()

  model_input = keras.layers.Input(tensor=img)
  model_output = model_layers(model_input)
  train_model = keras.models.Model(inputs=model_input, outputs=model_output)
  train_model.compile(optimizer=keras.optimizers.Adam(), # Alternative: RMSprop(lr=2e-3, decay=1e-5),
                      loss='mean_squared_error',
                      metrics=['mean_squared_error'],
                      target_tensors=[joints])
  return train_model

def main():
  parser = argparse.ArgumentParser(description='Trains a model from an input .tfrecord')
  parser.add_argument('--input', type=str, help='input .tfrecord file')
  parser.add_argument('--output', type=str, help='path to saved model destination')
  parser.add_argument('--logdir', type=str, help='output log directory')
  args = parser.parse_args()

  sess = K.get_session()

  print 'Loading features from', args.input
  features = dataset.load_features([args.input])
  features = features.shuffle(buffer_size=BUFFER_SIZE)
  data = {'test': features.take(TEST_HOLDOUT), 'train': features.skip(TEST_HOLDOUT)}

  print 'Constructing train and test models'
  train_model = _construct_model(data['train'])
  train_model.summary()
  test_model = _construct_model(data['test']) # used for validation during training

  print 'Fitting model (optimal: <0.0001 MSE)'
  coord = tf.train.Coordinator()
  threads = tf.train.start_queue_runners(sess, coord)
  tbCallBack = keras.callbacks.TensorBoard(log_dir=args.logdir, histogram_freq=0, write_graph=True, write_images=True)
  train_model.fit(
      epochs=EPOCHS,
      steps_per_epoch=int(np.ceil(NUM_EXAMPLES / float(BATCH_SIZE))),
      callbacks=[EvaluateInputTensor(test_model, steps=10), tbCallBack])

  print 'Saving model and weights to', args.output
  tf.train.Saver(tf.trainable_variables()).save(sess, args.output + '/saved')
  with open(args.output + '/model.json', 'w') as f:
    f.write(train_model.to_json())
  train_model.save_weights(args.output + '/weights.h5')


  print 'Cleaning up session'
  coord.request_stop()
  coord.join(threads)
  K.clear_session()

  print 'Done'
