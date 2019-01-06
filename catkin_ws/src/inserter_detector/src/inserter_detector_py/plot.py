from PIL import Image, ImageDraw
from inserter_detector_py import convert
import math
import numpy as np

CELL_LENGTH = 8
SCALE_FACTOR = convert.IMG_SIZE * 0.7

def newCanvas():
  return Image.new('RGB', (convert.IMG_SIZE * CELL_LENGTH, convert.IMG_SIZE * CELL_LENGTH), (128, 128, 128))

def _idxOffset(idx):
  return ((idx % CELL_LENGTH) * convert.IMG_SIZE, int(idx / CELL_LENGTH) * convert.IMG_SIZE)

def pasteImage(canvas, idx, img):
  img = Image.fromarray(img, 'L')
  canvas.paste(img, _idxOffset(idx))

def drawInference(canvas, idx, theta, extension, color):
  offset = _idxOffset(idx)
  center = (offset[0] + convert.IMG_SIZE//2, offset[1] + convert.IMG_SIZE//2)
  # Scale up as joint state is 0.1-1.1 normally
  theta = theta - (np.pi / 2)

  wV = (
    math.cos(theta) * extension * SCALE_FACTOR,
    -math.sin(theta) * extension * SCALE_FACTOR, # 0,0 is top left of image, so need to invert Y
  )
  draw = ImageDraw.Draw(canvas)
  coords = [
    (center[0], center[1]), # (round(center[0] - wV[0]/2), round(center[1] - wV[1]/2)),
    (round(center[0] + wV[0]/2), round(center[1] + wV[1]/2)),
  ]
  draw.line(coords, fill=color)
