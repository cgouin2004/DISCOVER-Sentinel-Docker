import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
import rospy
import math
from connorscode.msg import AiDetection

confidence_threshold = .5 # confidence score to consider detection
filter_detections = 3 # number of frame readings per inference
fps = 5 # how many inferences the program will make per second
filter_valid = .5 # distance (m) allowed in filter to be considered valid

def callback(data):
    global kp, torso_size, middle, detection
    # Store the received data in the global variable
    if data.class_id == 605:
        kp.update({
            'frame_id': data.frame_id,
            '6_x': data.x_min,
            '6_y': data.y_min,
            '5_x': data.x_max,
            '5_y': data.y_max,
            'confidence': data.detection_confidence
        })
        middle = (data.x_min+data.x_max)/2 # calculates middle point of body on x-axis
        # rospy.loginfo("605: %d %d %d %d at frame: %d", kp['6_x'], kp['6_y'], kp['5_x'], kp['5_y'], kp['frame_id'])
    elif ((data.class_id == 1112) and (kp['frame_id']) == data.frame_id):
        kp.update({
            '11_x': data.x_min,
            '11_y': data.y_min,
            '12_x': data.x_max,
            '12_y': data.y_max,
        })
        if (data.detection_confidence < kp['confidence']):
            kp['confidence'] = data.detection_confidence

        torso_size = 0.5 * abs(
            kp['5_y'] * kp['11_x'] - kp['5_x'] * kp['11_y'] +
            kp['11_y'] * kp['12_x'] - kp['11_x'] * kp['12_y'] +
            kp['12_y'] * kp['6_x'] - kp['12_x'] * kp['6_y'] +
            kp['6_y'] * kp['5_x'] - kp['6_x'] * kp['5_y']
        )  # Calculates quadrilateral given coordinates of vertices
        max_x=1024
        # max_y=768
        FOV = 70
        mp = max_x/2 # placeholder for true midpoint
        yaw = FOV*((mp-middle)/max_x) # placeholder equation
        distance = ((1000 / (math.sqrt(torso_size+1000)))-0.1)*0.305
        if kp['confidence'] >= confidence_threshold:
            for i in range(filter_detections-1):
                detection[i] = detection[i+1]
            detection[filter_detections - 1] = {'distance': distance, 'yaw': yaw}

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/tflite_data', AiDetection, callback)

if __name__ == '__main__':
    global kp, torso_size
    kp = {'frame_id': -1}
    detection = [None] * filter_detections
    listener()
    # Initialize global variables

    # Example usage of get_torso_size function

    x = np.linspace(0, 10, 10)+3
    y = np.zeros(10)
    print("Starting in 5 seconds")
    rospy.sleep(5)
    while detection[0] is None or detection[1] is None or detection[2] is None:
        rospy.sleep(1/fps)
   

    while True:
        valid = 0
        while valid == 0: # filter
            if abs(detection[0]['distance']-detection[1]['distance']) < filter_valid and abs(detection[0]['distance']-detection[2]['distance']) < filter_valid and abs(detection[1]['distance']-detection[2]['distance']) < filter_valid:
                valid = 1
            print('0')
        distance_t = detection[2]['distance']
        yaw_t = detection[2]['yaw']
        if kp['confidence'] >= confidence_threshold:
            print(distance_t,yaw_t)
        rospy.sleep(1/fps)
        