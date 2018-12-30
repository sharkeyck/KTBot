from keras.models import model_from_json
import keras
import convert
import argparse
import numpy as np
import math
from inserter_detector import dataset

def model_layers(input_layer):
  KERNEL_SIZE = (2, 2)
  x = keras.layers.Conv2D(16, kernel_size=KERNEL_SIZE,
                 activation='relu',
                 padding='same',
                 input_shape=(convert.IMG_SIZE, convert.IMG_SIZE, 1))(input_layer)
  x = keras.layers.AveragePooling2D(pool_size=KERNEL_SIZE)(x)
  x = keras.layers.Conv2D(32, kernel_size=KERNEL_SIZE,
                 activation='relu',
                 padding='same')(x)
  x = keras.layers.AveragePooling2D(pool_size=KERNEL_SIZE)(x)
  x = keras.layers.Conv2D(64, kernel_size=KERNEL_SIZE,
                 activation='relu',
                 padding='same')(x)
  x = keras.layers.AveragePooling2D(pool_size=KERNEL_SIZE)(x)
  x = keras.layers.Conv2D(128, kernel_size=KERNEL_SIZE,
                 activation='relu',
                 padding='same')(x)
  x = keras.layers.AveragePooling2D(pool_size=KERNEL_SIZE)(x)
  x = keras.layers.Conv2D(256, kernel_size=KERNEL_SIZE,
                 activation='relu',
                 padding='same')(x)
  x = keras.layers.AveragePooling2D(pool_size=KERNEL_SIZE)(x)
  x = keras.layers.Conv2D(512, kernel_size=KERNEL_SIZE,
                 activation='relu',
                 padding='same')(x)
  x = keras.layers.Flatten()(x)
  x = keras.layers.Dense(400, activation='relu')(x)
  x = keras.layers.Dropout(0.25)(x)
  x = keras.layers.Dense(128, activation='relu')(x)
  x = keras.layers.Dropout(0.25)(x)
  x = keras.layers.Dense(3, activation='linear')(x)

  # Best MSE loss in test: 0.0008516441390383989
  return x

class Model:
  def __init__(self, model_json_path, weights_path):
    self.model = self.load(model_json_path, weights_path)

  def load(self, model_json_path, weights_path):
    json_file = open(model_json_path, 'r')
    model_json = json_file.read()
    json_file.close()
    model = model_from_json(model_json)
    model.load_weights(weights_path)
    return model

  def predict_joint_states(self, normalized_clouds):
    images = np.array([convert.cloud_to_image(convert.flattened_list_to_cloud(c)) for c in normalized_clouds])
    prediction = self.model.predict(images)
    return [convert.prediction_to_joint_state(p) for p in prediction]

def main():
  parser = argparse.ArgumentParser(description='Does a few example predictions from a TF.Record file')
  parser.add_argument('--tfrecord_path', type=str, help='input .tfrecord file')
  parser.add_argument('--model_json_path', type=str, help='path to saved model json file')
  parser.add_argument('--weights_path', type=str, help='path to saved model weights')
  parser.add_argument('--num_examples', type=int, help='number of examples to evaluate', default=5)
  args = parser.parse_args()

  print("Loading model from", args.model_json_path, ", weights from", args.weights_path)
  detector = Model(args.model_json_path, args.weights_path)
  print("Model loaded")

  print("Loading", args.num_examples, "examples from", args.tfrecord_path)
  examples = dataset.get_feature_samples([args.tfrecord_path], args.num_examples)
  clouds = [v[0] for v in examples]
  want = [convert.prediction_to_joint_state(v[2]) for v in examples]

  print("Doing prediction")
  got = detector.predict_joint_states(clouds)

  print("Results:")
  def _norm_angle(theta):
    if theta < 0:
      return theta + 2*np.pi
    return theta
  for i, (g, w) in enumerate(zip(got, want)):
    wT = _norm_angle(w.position[0])
    gT = _norm_angle(g.position[0])
    # Distance between angles: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    dT = abs(math.atan2(math.sin(wT-gT), math.cos(wT-gT)))

    wE = w.position[1]
    gE = g.position[1]
    dE = abs(gE-wE)

    print("Example {0}\t (want, got, delta): angle ({1:.2f}, {2:.2f}, {3:.2f}) extension ({4:.2f}, {5:.2f}, {6:.2f})".format(i, wT, gT, dT, wE, gE, dE))
