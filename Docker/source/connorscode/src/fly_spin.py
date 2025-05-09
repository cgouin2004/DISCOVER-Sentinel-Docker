# Import necessary libraries
from SGFlightKitCore import FlightManager
import rospy
from mavros_msgs.msg import PositionTarget
import math

# Define the flight altitude constant
FLIGHT_ALT = 1
fps = 1

# Define the CubeCommander class
class CubeCommander:
    def __init__(self):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()
        # Wait for 2 seconds before sending the setpoint

        self.flight_manager.wait(2)

        self.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT)
        print("Initialize starting position (0,0,FLIGHT_ALT)")
       
        # Define the waypoints for the cube flight path

    def __del__(self):
        self.flight_manager = None

# Main entry point of the script
if __name__ == "__main__":

    print('Initializing Cube Commander')
    # Initialize the CubeCommander
    demo = CubeCommander()

    print('Waiting for offboard mode!')
    while (not rospy.is_shutdown() and not demo.flight_manager.is_vehicle_mode_offboard()):
        demo.flight_manager.wait(1)
        pass

    if rospy.is_shutdown():
        rospy.signal_shutdown()
        exit(1)
    print('Offboard Mode Detected: Waiting 5 seconds')
    demo.flight_manager.wait(5)
    print('Beginning sending waypoints!')
    yaw = math.pi/2
    while True:
        yaw = yaw + (math.pi/2)
        demo.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT, yaw)
        print(yaw)
        rospy.sleep(1/fps)
    
    print('finished program. Exiting')
    rospy.signal_shutdown()
    exit(1)
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #     rospy.spin()
