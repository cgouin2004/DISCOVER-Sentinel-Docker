# Import TF and TF Hub libraries.
import tensorflow as tf
import cv2
from matplotlib import image 
from matplotlib import pyplot as plt 

cam = cv2.VideoCapture(0)

while (True):
    
    result, image = cam.read()
    image_path = 'image.jpg'
    if result:
        #cv2.imshow("image", image)
        cv2.imwrite(image_path,image)
    # Load the input image.
    
    image = tf.io.read_file(image_path)
    image = tf.compat.v1.image.decode_jpeg(image)
    image = tf.expand_dims(image, axis=0)
    # Resize and pad the image to keep the aspect ratio and fit the expected size.
    image = tf.image.resize_with_pad(image, 192, 192)

    # Initialize the TFLite interpreter
    model_path = '3.tflite'
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    # TF Lite format expects tensor type of float32.
    input_image = tf.cast(image, dtype=tf.float32)
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    interpreter.set_tensor(input_details[0]['index'], input_image.numpy())

    interpreter.invoke()

    # Output is a [1, 1, 17, 3] numpy array.
    keypoints_with_scores = interpreter.get_tensor(output_details[0]['index'])
    kp=keypoints_with_scores[0][0]
    torso_size = 0.5*(kp[5][1]*kp[11][0]-kp[5][0]*kp[11][1]+kp[11][1]*kp[12][0]-kp[11][0]*kp[12][1]+kp[12][1]*kp[6][0]-kp[12][0]*kp[6][1]+kp[6][1]*kp[5][0]-kp[6][0]*kp[5][1])
    print(f"Torso size: {torso_size}")
    print(f"Torso position {keypoints_with_scores[0][0][0][1]}")

    # to plot keypoints
    # data = cv2.imread('image.jpg') 
    
    # plt.plot(keypoints_with_scores[0][0][0][1]*numx, keypoints_with_scores[0][0][0][0]*numy, marker='v', color="white") 
    # plt.plot(keypoints_with_scores[0][0][1][1]*numx, keypoints_with_scores[0][0][1][0]*numy, marker='v', color="white") 
    # plt.plot(keypoints_with_scores[0][0][2][1]*numx, keypoints_with_scores[0][0][2][0]*numy, marker='v', color="white") 
    # plt.plot(keypoints_with_scores[0][0][3][1]*numx, keypoints_with_scores[0][0][3][0]*numy, marker='v', color="white") 
    # plt.plot(keypoints_with_scores[0][0][4][1]*numx, keypoints_with_scores[0][0][4][0]*numy, marker='v', color="white") 
    # plt.plot(keypoints_with_scores[0][0][5][1]*numx, keypoints_with_scores[0][0][5][0]*numy, marker='v', color="white") 
    # plt.plot(keypoints_with_scores[0][0][6][1]*numx, keypoints_with_scores[0][0][6][0]*numy, marker='v', color="white") 
    # plt.imshow(data) 
    # plt.savefig('plottedimg.png')
    # plt.close('all')

    