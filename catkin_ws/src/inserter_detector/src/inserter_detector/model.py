from keras.models import model_from_json
import keras
import convert
import argparse
import numpy as np
import math
from inserter_detector import dataset
from PIL import Image, ImageDraw

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

  def _pasteSerialImage(canvas, serial_img, offset):
    img = Image.fromarray(convert.serial_image_to_2d(serial_img), 'L')
    canvas.paste(img, offset)

  def _drawInference(canvas, theta, extension, offset, color):
    center = (offset[0] + convert.IMG_SIZE//2, offset[1] + convert.IMG_SIZE//2)
    wV = (-math.cos(theta) * extension, -math.sin(theta * extension)) # -sin and -cos because origin is top left
    wV = (wV[0] * convert.IMG_SIZE, wV[1] * convert.IMG_SIZE) # Scale up as joint state is 0.1-1.1 normally
    draw = ImageDraw.Draw(canvas)
    draw.line((
      center[0] - wV[0]//2,
      center[1] - wV[1]//2,
      center[0] + wV[0]//2,
      center[1] + wV[1]//2,
    ), fill=color)

  cell_length = 8
  canvas = Image.new('RGB', (convert.IMG_SIZE * cell_length, convert.IMG_SIZE * cell_length), (128, 128, 128))
  for r in results:
    print("Example {i}\t (want, got, delta): angle ({wT:.2f}, {gT:.2f}, {dT:.2f}) extension ({wE:.2f}, {gE:.2f}, {dE:.2f})".format(**r))

    offset = ((r['i'] % cell_length) * convert.IMG_SIZE, int(r['i'] / cell_length) * convert.IMG_SIZE)
    _pasteSerialImage(canvas, examples[r['i']][1], offset)
    _drawInference(canvas, r['wT'], r['wE'], offset, (255, 0, 0))
    _drawInference(canvas, r['gT'], r['gE'], offset, (0, 255, 0))

  canvas = canvas.resize((1024,1024))
  canvas.show()

