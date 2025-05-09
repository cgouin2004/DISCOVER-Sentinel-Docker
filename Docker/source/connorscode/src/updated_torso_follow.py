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
fps = 8 # how many inferences the program will make per second
max_d = 0.5 # maximum distance the drone will move with any 1 inference (meters)
confidence_threshold = .4 # confidence score to consider detection
filter_detections = 3 # number of frame readings per inference
filter_valid = .5 # distance (m) allowed in filter to be considered valid
distance_threshold = .1 # maximum distance drone will idle
yaw_threshold = 5 # maximum yaw drone will idle from center (degrees)


# Define the FollowCommander class
class FollowCommander:
    def __init__(self):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()
        print("Initializing flight manager")
        # Wait for 2 seconds before sending the setpoint

        self.flight_manager.wait(2)

        self.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT)
        print("Initialize starting position (0,0,FLIGHT_ALT)")

       
        # Define the waypoints for the cube flight path

    def __del__(self):
        self.flight_manager = None

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


def pose_listener():
    if not rospy.core.is_initialized():
        rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber('/tflite_data', AiDetection, callback)


if __name__ == '__main__':
    global kp, torso_size, detection
    
    kp = {'frame_id': -1}
    detection = [None] * filter_detections
    print('Initializing Follow Commander')
    # Initialize the FollowCommander
    demo = FollowCommander()
    print('Waiting for offboard mode!')
    pose_listener()
    while (not rospy.is_shutdown() and not demo.flight_manager.is_vehicle_mode_offboard()):
        demo.flight_manager.wait(1)
        pass

    if rospy.is_shutdown():
        rospy.signal_shutdown()
        exit(1)

    print('Offboard Mode Detected: Waiting 5 seconds')
    demo.flight_manager.wait(5)
    print('Retrieving starting distance')

    while detection[0] is None or detection[1] is None or detection[2] is None:
        demo.flight_manager.wait(1/fps)
   
    valid = 0
    while valid == 0: # filter
        if abs(detection[0]['distance']-detection[1]['distance']) < filter_valid and abs(detection[0]['distance']-detection[2]['distance']) < filter_valid and abs(detection[1]['distance']-detection[2]['distance']) < filter_valid:
            valid = 1
        print('0')
    distance_start = detection[2]['distance']
    print(f'Distance Start: {distance_start} meters')

    while 1:
        valid = 0
        frame = -1
        last_frame = -1
        d = np.empty(filter_detections)
        
        valid = 0
        while valid == 0: # filter
            if abs(detection[0]['distance']-detection[1]['distance']) < filter_valid and abs(detection[0]['distance']-detection[2]['distance']) < filter_valid and abs(detection[1]['distance']-detection[2]['distance']) < filter_valid and kp['confidence'] >= confidence_threshold:
                valid = 1
            else:
                print('0')
            rospy.sleep(1/fps)
        distance_t = detection[2]['distance'] - distance_start
        dyaw_t = detection[2]['yaw']
        print(f'Distance = {distance_t}')

        if abs(distance_t) > distance_threshold or abs(dyaw_t) > yaw_threshold:

            if abs(distance_t) > max_d:
                if distance_t > 0:
                    distance_t = max_d
                if distance_t < 0:
                    distance_t = -1 * max_d

            ryaw_t = dyaw_t*(math.pi/180)
            ryaw = (demo.flight_manager.current_orientation.get('yaw'))+ryaw_t
            dyaw = ryaw*(180/math.pi)
            x = math.cos(ryaw)*distance_t + demo.flight_manager.current_local_loc.pose.position.x
            y = math.sin(ryaw)*distance_t + demo.flight_manager.current_local_loc.pose.position.y
            print(f'Sending Waypoint! Distance: {distance_t} Yaw: {dyaw_t} to X: {x} Y: {y} Yaw:{dyaw}')
            demo.flight_manager.fly_to_position(x=x, y=y, z=FLIGHT_ALT, yaw=ryaw)

        rospy.sleep(1/fps)

    # Initialize global variables

    

