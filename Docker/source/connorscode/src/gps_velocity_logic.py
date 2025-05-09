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

        self.flight_manager.wait(.5)
       
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
        lat=demo.flight_manager.current_global_loc.latitude
        long=demo.flight_manager.current_global_loc.longitude
        alt=demo.flight_manager.current_global_loc.altitude
        yaw=demo.flight_manager.current_orientation.get('yaw')*180/math.pi
        print(f'latitude position: {lat} longitude position: {long} altitude position: {alt} yaw: {yaw}\n')
        (distance, yaw2) = demo.flight_manager.distance_and_yaw(lat1=lat, long1=long, lat2=34.68448094853823, long2=-82.81711687595892)
        if (distance > 2):
            print(f'Move {distance} meters in direction {yaw2} degrees\n')
        else: print('WAYPOINT REACHED\n')
        rospy.sleep(0.1)
        

    if rospy.is_shutdown():
        rospy.signal_shutdown()
        exit(1)
    
    print('finished program. Exiting')
    rospy.signal_shutdown('Exiting Successfully')
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #    rospy.spin()
