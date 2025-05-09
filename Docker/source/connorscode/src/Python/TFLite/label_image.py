import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
import math
import urllib.request

def detect(interpreter, input_tensor):
  """Runs detection on an input image.

  Args:
    interpreter: tf.lite.Interpreter
    input_tensor: A [1, input_height, input_width, 3] Tensor of type tf.float32.
      input_size is specified when converting the model to TFLite.

  Returns:
    A tensor of shape [1, 6, 56].
  """

  input_details = interpreter.get_input_details()
  output_details = interpreter.get_output_details()

  is_dynamic_shape_model = input_details[0]['shape_signature'][2] == -1
  if is_dynamic_shape_model:
    input_tensor_index = input_details[0]['index']
    input_shape = input_tensor.shape
    interpreter.resize_tensor_input(
        input_tensor_index, input_shape, strict=True)
  interpreter.allocate_tensors()

  interpreter.set_tensor(input_details[0]['index'], input_tensor.numpy())

  interpreter.invoke()

  keypoints_with_scores = interpreter.get_tensor(output_details[0]['index'])
  return keypoints_with_scores

def keep_aspect_ratio_resizer(image, target_size):
  """Resizes the image.

  The function resizes the image such that its longer side matches the required
  target_size while keeping the image aspect ratio. Note that the resizes image
  is padded such that both height and width are a multiple of 32, which is
  required by the model.
  """
  _, height, width, _ = image.shape
  if height > width:
    scale = float(target_size / height)
    target_height = target_size
    scaled_width = math.ceil(width * scale)
    image = tf.image.resize(image, [target_height, scaled_width])
    target_width = int(math.ceil(scaled_width / 32) * 32)
  else:
    scale = float(target_size / width)
    target_width = target_size
    scaled_height = math.ceil(height * scale)
    image = tf.image.resize(image, [scaled_height, target_width])
    target_height = int(math.ceil(scaled_height / 32) * 32)
  image = tf.image.pad_to_bounding_box(image, 0, 0, target_height, target_width)
  return (image,  (target_height, target_width))


# Load the input image.
input_size = 256
image_path = '2024-05-13-094023.jpg'
image = tf.io.read_file(image_path)
image = tf.compat.v1.image.decode_jpeg(image)
image = tf.expand_dims(image, axis=0)

# Resize and pad the image to keep the aspect ratio and fit the expected size.
resized_image, image_shape = keep_aspect_ratio_resizer(image, input_size)
image_tensor = tf.cast(resized_image, dtype=tf.uint8)

model_path = '1.tflite'
interpreter = tf.lite.Interpreter(model_path=model_path)

# Output: [1, 6, 56] tensor that contains keypoints/bbox/scores.
keypoints_with_scores = detect(
    interpreter, tf.cast(image_tensor, dtype=tf.uint8))

print(keypoints_with_scores[0][0])
