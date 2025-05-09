# Import necessary libraries
from SGFlightKitCore import FlightManager
import rospy
import math
from mavros_msgs.msg import PositionTarget

# Define the flight altitude constant
FLIGHT_ALT = 1

# Define the CubeCommander class
class CubeCommander:
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

# Main entry point of the script
if __name__ == "__main__":


    print('Initializing Cube Commander')
    # Initialize the CubeCommander
    demo = CubeCommander()

    print('Waiting for offboard mode!')
    while (not rospy.is_shutdown()):
        demo.flight_manager.wait(1)
        x=demo.flight_manager.current_local_loc.pose.position.x
        y=demo.flight_manager.current_local_loc.pose.position.y
        z=demo.flight_manager.current_local_loc.pose.position.z
        yaw=demo.flight_manager.current_orientation.get('yaw')*180/math.pi
        print(f'x position: {x} y position: {y} z position: {z} yaw: {yaw}')
        rospy.sleep(0.1)
        

    if rospy.is_shutdown():
        rospy.signal_shutdown()
        exit(1)
    
    print('finished program. Exiting')
    rospy.signal_shutdown('Exiting Successfully')
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #    rospy.spin()
