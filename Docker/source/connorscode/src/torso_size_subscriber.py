import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
import rospy
from connorscode.msg import AiDetection


def callback(data):
    global kp, torso_size
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
        # rospy.loginfo("605: %d %d %d %d at frame: %d", kp['6_x'], kp['6_y'], kp['5_x'], kp['5_y'], kp['frame_id'])
    elif data.class_id == 1112:
        kp.update({
            '11_x': data.x_min,
            '11_y': data.y_min,
            '12_x': data.x_max,
            '12_y': data.y_max,
        })
        torso_size = 0.5 * abs(
            kp['5_y'] * kp['11_x'] - kp['5_x'] * kp['11_y'] +
            kp['11_y'] * kp['12_x'] - kp['11_x'] * kp['12_y'] +
            kp['12_y'] * kp['6_x'] - kp['12_x'] * kp['6_y'] +
            kp['6_y'] * kp['5_x'] - kp['6_x'] * kp['5_y']
        )  # Calculates quadrilateral given coordinates of vertices

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/tflite_data', AiDetection, callback)

def get_torso_size():
    global kp, torso_size
    while torso_size == 0:
        listener()
    return torso_size

if __name__ == '__main__':
    global kp, torso_size
    kp = {'frame_id': -1}
    torso_size = 0
    # Initialize global variables

    # Example usage of get_torso_size function

    x = np.linspace(0, 10, 10)+3
    y = np.zeros(10)
    print("Starting in 5 seconds")
    rospy.sleep(5)
    for i in range (10):
        y[i] = get_torso_size()
        print(f"{x[i]} , {y[i]}")
        print("Waiting 5 seconds")
        rospy.sleep(5)
        
    
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, label='Torso Size vs. Distance')
    plt.title('Torso Size vs. Distance')
    plt.xlabel('Distance (feet)')
    plt.ylabel('Torso Size')
    plt.grid(True)
    plt.savefig('line_plot.png')
    plt.close()
