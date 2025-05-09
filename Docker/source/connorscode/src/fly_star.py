# Import necessary libraries
from SGFlightKitCore import FlightManager
import rospy
from mavros_msgs.msg import PositionTarget

# Define the flight altitude constant
FLIGHT_ALT = 1

# Define the StarCommander class
class StarCommander:
    def __init__(self, waypoints):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()
        # Wait for 2 seconds before sending the setpoint


        self.waypoints = waypoints
        self.flight_manager.wait(2)

        self.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT)
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
        self.flight_manager.fly_to_position(self.flight_manager.current_local_loc.pose.position.x + waypoint[0], self.flight_manager.current_local_loc.pose.position.y + waypoint [1], self.flight_manager.current_local_loc.pose.position.z + waypoint[2])
        print("Flying to:", waypoint)
        # Increment the current waypoint index
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    def __del__(self):
        self.flight_manager = None

# Main entry point of the script
if __name__ == "__main__":


    waypoints = [
            (0.5, 1.5, 0.0),
            (0.5, -1.5, 0.0),
            (-1.25, 1.0, 0.0),
            (1.5, 0.0, 0.0),
            (-1.25, -1.0, 0.0),
        ]

    print('Initializing Star Commander')
    # Initialize the StarCommander
    demo = StarCommander(waypoints=waypoints)

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
    for id, i in enumerate(range(len(waypoints))):
        print(f'Goint to waypoint #{id}')
        demo.fly_to_next_waypoint()
        rospy.sleep(8)
    
    print('finished program. Exiting')
    rospy.signal_shutdown()
    exit(1)
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #     rospy.spin()
