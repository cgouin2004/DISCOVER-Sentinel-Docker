# Import necessary libraries
from SGFlightKitCore import FlightManager
import rospy
from mavros_msgs.msg import PositionTarget

# Define the flight altitude constant
FLIGHT_ALT = 1

# Define the CubeCommander class
class CubeCommander:
    def __init__(self, waypoints):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()
        print("Initializing flight manager")
        # Wait for 2 seconds before sending the setpoint


        self.waypoints = waypoints
        self.flight_manager.wait(2)

        #self.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT)
        print("Initialize starting position (0,0,FLIGHT_ALT)")

       
        # Define the waypoints for the cube flight path

        self.current_waypoint_index = 0

    def fly_to_next_waypoint(self):
        while (not rospy.is_shutdown() and not self.flight_manager.is_vehicle_mode_offboard()):
            self.flight_manager.wait(1)
            pass

        if rospy.is_shutdown():
            return
        
        # Get the next waypoint from the list
        waypoint = self.waypoints[self.current_waypoint_index]
        # Fly to the waypoint
        # self.flight_manager.fly_to_position(*waypoint)
        print("Flying to:", waypoint)

        # Increment the current waypoint index
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    def __del__(self):
        self.flight_manager = None

# Main entry point of the script
if __name__ == "__main__":


    waypoints = [
            (0.5, 0.5, FLIGHT_ALT),
            (0.5, -0.5, FLIGHT_ALT),
            (-0.5, -0.5, FLIGHT_ALT),
            (-0.5, 0.5, FLIGHT_ALT),
            (0.5, 0.5, FLIGHT_ALT),
            (0.5, 0.5, FLIGHT_ALT+1),
            (0.5, -0.5, FLIGHT_ALT+1),
            (-0.5, -0.5, FLIGHT_ALT+1),
            (-0.5, 0.5, FLIGHT_ALT+1),
            (0.5, 0.5, FLIGHT_ALT+1)
        ]

    print('Initializing Cube Commander')
    # Initialize the CubeCommander
    demo = CubeCommander(waypoints=waypoints)

    print('Waiting for offboard mode!')
    while (not rospy.is_shutdown() and not demo.flight_manager.is_vehicle_mode_offboard()):
        demo.flight_manager.wait(1)
        pass

    if rospy.is_shutdown():
        rospy.signal_shutdown()
        exit(1)

    print('Beginning sending waypoints!')
    for id, i in enumerate(range(len(waypoints))):
        print(f'Goint to waypoint #{id}')
        demo.fly_to_next_waypoint()
        rospy.sleep(2)
    
    print('finished program. Exiting')
    rospy.signal_shutdown('Exiting Successfully')
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #    rospy.spin()
