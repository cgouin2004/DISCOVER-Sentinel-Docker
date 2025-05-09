# Import necessary libraries
import SGFlightKitCore
from SGFlightKitCore import FlightManager
import rospy
import math
from mavros_msgs.msg import PositionTarget
from SGVideo import VideoRecorder

# Define the flight altitude constant
FLIGHT_ALT = 1

TOPIC_NAME   = '/hires_small_color'  # image topic to record
DURATION     = 60.0         # seconds to record
OUTPUT_FILE  = '/home/ros/ros_ws/src/connorscode/video/color_mission_test_video.mp4'
FRAME_RATE   = 8.0         # frames per second

# Define the CubeCommander class
class FlightCommander:
    def __init__(self):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()
        # Wait for 2 seconds before sending the setpoint

        self.flight_manager.wait(2)
        self.flight_manager.fly_to_position(velx=0.0, vely=0.0, velz=1.0)
        print("Initialize starting pose")

        print('Waiting for offboard mode')

    def __del__(self):
        self.flight_manager = None

# Main entry point of the script
if __name__ == "__main__":

    print('Initializing Cube Commander')
    # Initialize the CubeCommander
    demo = FlightCommander()

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
    
    demo.flight_manager.fly_mission(filename='mission.json')
    recorder.stop()
    rospy.loginfo("Recording complete.")
    print('finished program. Exiting')
    rospy.signal_shutdown()
    exit(1)
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #     rospy.spin()
