# Import necessary libraries
from SGFlightKitCore import FlightManager
import rospy
from mavros_msgs.msg import PositionTarget
import math
import tensorflow as tf
import cv2
import image_saver as im

# Define the flight altitude constant
FLIGHT_ALT = 1
FOVD = 67.38 # degrees
FOVR = 1.176 # radians
image_path = 'image.jpg'

# Define the FollowCommander class
class FollowCommander:
    def __init__(self):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()
        # Wait for 2 seconds before sending the setpoint

        self.flight_manager.wait(2)

        self.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT)
        print("Initialize starting position (0,0,FLIGHT_ALT)")
       
        # Define the waypoints for the cube flight path

        self.current_waypoint_index = 0


    def __del__(self):
        self.flight_manager = None

    def obtain_torso_size (self):
        im.main()
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
        torso_size = 0.5*(kp[5][1]*kp[11][0]-kp[5][0]*kp[11][1]+kp[11][1]*kp[12][0]-kp[11][0]*kp[12][1]+kp[12][1]*kp[6][0]-kp[12][0]*kp[6][1]+kp[6][1]*kp[5][0]-kp[6][0]*kp[5][1]) # calculaltes quatrilateral given coordinates of verticies
        return torso_size

    def obtain_torso_position (self):     
        im.main()
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
        return keypoints_with_scores[0][0][0][1]
    
# Main entry point of the script
if __name__ == "__main__":

    
    print('Initializing Cube Commander')
    # Initialize the FollowCommander
    demo = FollowCommander()
    interval = .1 # how long it allows for flight before checking for another change in position
    size_buffer = 10 # controls idle distance (higher = more precise)
    position_buffer = 10 # controls idle rotation (lower = more precise)
    maxd = .5 # maximum distance change allowed per cycle
    filtern = 5 # number of pictures to reduce bad detections
    position_center = .5
    torso_total=0
    ddistance=0
    dyaw=0

    print('Waiting for offboard mode!')
    while (not rospy.is_shutdown() and not demo.flight_manager.is_vehicle_mode_offboard()):
        demo.flight_manager.wait(1)

    if rospy.is_shutdown():
        rospy.signal_shutdown()
        exit(1)
    print('Offboard Mode Detected: Waiting 5 seconds')
    demo.flight_manager.wait(5)
    print('Initializing starting distance')
    torso_start = demo.obtain_torso_size() # captures size of torso at start (for reference)
    torso_size=torso_start
    print('Beginning following sequence!')
    while (1):
        torso_size = demo.obtain_torso_size()
        torso_position = demo.obtain_torso_position()
        if (abs(torso_size-torso_start) > (torso_start/size_buffer) or abs(torso_position-position_center) > position_buffer): # only move if too close or too far / not pointing at target
            x = demo.flight_manager.current_local_loc.pose.position.x
            y = demo.flight_manager.current_local_loc.pose.position.y
            if (torso_start-torso_size) < 0:
                ddistance=2**(torso_start-torso_size) # calculates how much the drone should move
            else:
                ddistance=(-torso_start+torso_size)*(1/2)*2
            if ddistance > maxd:
                ddistance = maxd
            dyaw = (position_center-torso_position)*FOVD # calculates how much the drone should rotate
        print(f"Go forward: {ddistance} meters || Yaw {dyaw} degrees")
        rospy.sleep(interval) # loops after set amount of time
            
    
    print('finished program. Exiting')
    rospy.signal_shutdown()
    exit(1)
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #     rospy.spin()