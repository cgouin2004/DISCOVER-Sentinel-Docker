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
        # Wait for 2 seconds before sending the setpoint

        self.flight_manager.wait(2)
        startx = self.flight_manager.current_local_loc.pose.position.x
        starty = self.flight_manager.current_local_loc.pose.position.y
        startz = self.flight_manager.current_local_loc.pose.position.z+2
        self.flight_manager.fly_to_position(velx=0, vely=0, velz=1.0)
        print("Initialize starting position (0,0,FLIGHT_ALT)")

        print('Waiting for offboard mode')

    def __del__(self):
        self.flight_manager = None

# Main entry point of the script
if __name__ == "__main__":

    print('Initializing Cube Commander')
    # Initialize the CubeCommander
    demo = CubeCommander()

    print('Waiting for offboard mode!')
    while (not rospy.is_shutdown() and not demo.flight_manager.is_vehicle_mode_offboard()):
        demo.flight_manager.wait(.1)

    if rospy.is_shutdown():
        rospy.signal_shutdown()
        exit(1)
    print('Offboard Mode Detected: Waiting 3 seconds')

    (startx, starty, startz) = demo.flight_manager.initialize_takeoff()

    demo.flight_manager.fly_simple(lateral_dist=30.0, yaw=0.0, speed=5.0)
    print('1')
    demo.flight_manager.fly_simple(lateral_dist=15.0, yaw=90.0, speed=1.0, height_diff=3)
    print('2')
    demo.flight_manager.fly_simple(lateral_dist=30.0, yaw=90.0, speed=5.0, height_diff=-3)
    print('3')
    demo.flight_manager.fly_simple(lateral_dist=15.0, yaw=90.0, speed=1.0)
    print('4')

    demo.flight_manager.initialize_landing(startx=startx, starty=starty, startz=startz)

    demo.flight_manager.wait(2)
    print('finished program. Exiting')
    rospy.signal_shutdown()
    exit(1)
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #     rospy.spin()
