# Import necessary libraries
import SGFlightKitCore
from SGFlightKitCore import FlightManager
import rospy
import math
from mavros_msgs.msg import PositionTarget
from SGVideo import VideoRecorder

# Define the flight altitude constant
FLIGHT_ALT = 1

TOPIC_NAME   = '/tracking'  # image topic to record
DURATION     = 60.0         # seconds to record
OUTPUT_FILE  = '/home/ros/ros_ws/src/connorscode/video/mission_test_video.mp4'
FRAME_RATE   = 5.0         # frames per second

# Define the CubeCommander class
class FlightCommander:
    def __init__(self):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()
        # Wait for 2 seconds before sending the setpoint

        self.flight_manager.wait(2)
        self.flight_manager.fly_to_position(velx=0.0, vely=0.0, velz=1.0)
        print("Initialize starting position (0,0,FLIGHT_ALT)")

        print('Waiting for offboard mode')

    def __del__(self):
        self.flight_manager = None

# Main entry point of the script
if __name__ == "__main__":

    print('Initializing Cube Commander')
    # Initialize the CubeCommander
    demo = FlightCommander()

    print('Waiting for offboard mode!')
    while (not rospy.is_shutdown() and not demo.flight_manager.is_vehicle_mode_offboard()):
        demo.flight_manager.wait(0.1)
        

    if rospy.is_shutdown():
        rospy.signal_shutdown()
        exit(1)
    print('Offboard Mode Detected: Taking off')
    

    # Instantiate recorder with constants
    recorder = VideoRecorder(
        topic_name=TOPIC_NAME,
        frame_rate=FRAME_RATE,
        output_file=OUTPUT_FILE
    )
    recorder.start()
    rospy.loginfo("Recording %s seconds from topic '%s' @ %.2f FPS to '%s'", 
                  DURATION, TOPIC_NAME, FRAME_RATE, OUTPUT_FILE)
    
    (startx, starty, startz) = demo.flight_manager.initialize_takeoff(delay = 5.0)

    demo.flight_manager.fly_gps_velocity(lat=34.58750273155081, long=-82.80774063317057, speed=1, dest_rel_alt=startz, delay=1)
    demo.flight_manager.fly_gps_velocity(lat=34.587415189177044, long=-82.80751959733547, speed=2, dest_rel_alt=startz, delay=2)
    demo.flight_manager.fly_gps_velocity(lat=34.58704073929809, long=-82.80776852849705, speed=3, dest_rel_alt=startz+4, delay=5)
    demo.flight_manager.fly_gps_velocity(lat=34.58705254282278, long=-82.80807319951298, speed=4, dest_rel_alt=startz, delay=0)
    demo.flight_manager.fly_gps_velocity(lat=34.58750273155081, long=-82.80774063317057, speed=5, dest_rel_alt=startz, delay=3)

    demo.flight_manager.initialize_landing(startx=startx, starty=starty, startz=startz)
    recorder.stop()
    rospy.loginfo("Recording complete.")
    print('finished program. Exiting')
    rospy.signal_shutdown()
    exit(1)
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #     rospy.spin()
