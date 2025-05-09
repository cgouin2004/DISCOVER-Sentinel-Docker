import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
from SGFlightKitCore import FlightManager
import rospy
import math
from mavros_msgs.msg import PositionTarget
from connorscode.msg import AiDetection

# Define the flight altitude constant
FLIGHT_ALT = 1

# Define the CubeCommander class
class FollowCommander:
    def __init__(self):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()
        print("Initializing flight manager")
        # Wait for 2 seconds before sending the setpoint

        self.flight_manager.wait(2)

        #self.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT)
        print("Initialize starting position (0,0,FLIGHT_ALT)")

       
        # Define the waypoints for the cube flight path

    def __del__(self):
        self.flight_manager = None

def callback(data):
    global kp, torso_size, middle
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

def listener():
    if not rospy.core.is_initialized():
        rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/tflite_data', AiDetection, callback)

def get_body_info():
    global kp, torso_size, middle
    torso_size = 0
    max_x=1024
    # max_y=768
    FOV = 60
    mp = max_x/2 # placeholder for true midpoint
    while torso_size == 0:
        listener()
    yaw = FOV*((mp-middle)/max_x) # placeholder equation
    return torso_size, yaw, kp['confidence']


def get_distance(size):
    distance = ((1000 / (math.sqrt(size+1000)))-0.1)*0.305 # placeholder equation
    return distance

if __name__ == '__main__':
    global kp, torso_size
    kp = {'frame_id': -1}
    print('Initializing Cube Commander')
    # Initialize the CubeCommander
    demo = FollowCommander()
    while 1:
        size, dyaw_t, conf = get_body_info()
        distance_t = get_distance(size)
        ryaw_t = dyaw_t*(math.pi/180)
        ryaw = (demo.flight_manager.current_orientation.get('yaw'))+ryaw_t
        dyaw = ryaw*(180/math.pi)
        x = math.cos(ryaw)*distance_t + demo.flight_manager.current_local_loc.pose.position.x
        y = math.sin(ryaw)*distance_t + demo.flight_manager.current_local_loc.pose.position.y
        print(f'Distance: {distance_t} Yaw: {dyaw_t} to X: {x} Y: {y} Yaw:{dyaw}')
        rospy.sleep(.5)

    # Initialize global variables

    

