from keras.models import model_from_json
import keras
import convert
import argparse
import numpy as np
import math
from inserter_detector_py import dataset, plot

def model_layers(input_layer):
  KERNEL_SIZE = (2, 2)
  x = keras.layers.Conv2D(16, kernel_size=KERNEL_SIZE,
                 activation='relu',
                 padding='same',
                 input_shape=(convert.IMG_SIZE, convert.IMG_SIZE, 1))(input_layer)
  x = keras.layers.AveragePooling2D(pool_size=KERNEL_SIZE)(x)
  x = keras.layers.Dropout(0.1)(x)
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
  x = keras.layers.Dropout(0.15)(x)
  x = keras.layers.Dense(1024, activation='relu')(x)
  x = keras.layers.Dropout(0.15)(x)
  x = keras.layers.Dense(3, activation='linear')(x)

  # Best MSE loss in test: 0.0008516441390383989
  return x

class Model:
  def __init__(self, model_json_path, weights_path):
    self.model = self.load(model_json_path, weights_path)
    self.model._make_predict_function()

  def load(self, model_json_path, weights_path):
    json_file = open(model_json_path, 'r')
    model_json = json_file.read()
    json_file.close()
    model = model_from_json(model_json)
    model.load_weights(weights_path)
    return model

  def predict_joint_states(self, images):
    prediction = self.model.predict(images)
    return [convert.prediction_to_joint_state(p) for p in prediction]

def main():
  parser = argparse.ArgumentParser(description='Does a few example predictions from a TF.Record file')
  parser.add_argument('--tfrecord_path', type=str, help='input .tfrecord file')
  parser.add_argument('--model_json_path', type=str, help='path to saved model json file')
  parser.add_argument('--weights_path', type=str, help='path to saved model weights')
  parser.add_argument('--num_examples', type=int, help='number of examples to evaluate', default=64)
  args = parser.parse_args()

  print("Loading model from", args.model_json_path, ", weights from", args.weights_path)
  detector = Model(args.model_json_path, args.weights_path)
  print("Model loaded")

  print("Loading", args.num_examples, "examples from", args.tfrecord_path)
  examples = dataset.get_feature_samples([args.tfrecord_path], args.num_examples)
  images = np.array([v[0] for v in examples])
  want = [convert.prediction_to_joint_state(v[1]) for v in examples]

  print("Doing prediction")
  got = detector.predict_joint_states(images)

  print("Results:")
  results = []
  for i, (g, w) in enumerate(zip(got, want)):
    r = {
      'i': i,
      'wT': w.position[0],
      'gT': g.position[0],
      'wE': w.position[1],
      'gE': g.position[1],
    }

    # Distance between angles: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    r['dT'] = abs(math.atan2(math.sin(r['wT']-r['gT']), math.cos(r['wT']-r['gT'])))
    r['dE'] = abs(r['gE']-r['wE'])

    results.append(r)

  canvas = plot.newCanvas()
  for r in results:
    print("Example {i}\t (want, got, delta): angle ({wT:.2f}, {gT:.2f}, {dT:.2f}) extension ({wE:.2f}, {gE:.2f}, {dE:.2f})".format(**r))
    plot.pasteImage(canvas, r['i'], examples[r['i']][0].reshape(convert.IMG_SIZE, convert.IMG_SIZE).astype(np.uint8))
    plot.drawInference(canvas, r['i'], r['wT'], r['wE'], (255, 0, 0))
    plot.drawInference(canvas, r['i'], r['gT'], r['gE'], (0, 255, 0))

  print("Avg error {aeT:.2f}deg, {aeE:.2f} extension".format(aeT=np.mean([r['dT'] for r in results]) * 180 / np.pi, aeE=np.mean([r['dE'] for r in results])))

  canvas = canvas.resize((1024,1024))
  canvas.show()

