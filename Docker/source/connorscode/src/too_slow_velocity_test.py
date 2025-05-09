# Import necessary libraries
from SGFlightKitCore import FlightManager
import rospy
import math
from mavros_msgs.msg import PositionTarget

# Define the flight altitude constant
FLIGHT_ALT = 1

# Define the CubeCommander class
class SpeedCommander:
    def __init__(self, waypoints, velocities):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()

        self.waypoints = waypoints

        self.velocities = velocities
        # Wait for 2 seconds before sending the setpoint
        self.flight_manager.wait(2)

        self.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT)

        while (not rospy.is_shutdown() and not self.flight_manager.is_vehicle_mode_offboard()):
            self.flight_manager.wait(1)
            pass

        if rospy.is_shutdown():
            return
        
        # Define the waypoints for the cube flight path

        self.current_waypoint_index = 0

    def fly_to_next_waypoint(self):
        # Get the next waypoint from the list

        waypoint = self.waypoints[self.current_waypoint_index+1] # saves waypoint to go to next
        pwaypoint = self.waypoints[self.current_waypoint_index] # saves waypoint drone is currently at
        distancex = waypoint[0]-pwaypoint[0] 
        distancey = waypoint[1]-pwaypoint[1]
        distancez = waypoint[2]-pwaypoint[2]
        distancet = math.sqrt(distancex**2+distancey**2+distancez**2) # calculates total distance drone is projected to travel

        # calculates x, y, z velocities based on their components of distance
        velx=self.velocities[self.current_waypoint_index]*(distancex/distancet)
        vely=self.velocities[self.current_waypoint_index]*(distancey/distancet)
        velz=self.velocities[self.current_waypoint_index]*(distancez/distancet)
        # Fly to the waypoint
        self.flight_manager.fly_to_position(*waypoint,velx=velx,vely=vely,velz=velz)
        # Increment the current waypoint index
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
 
    def go_to_first_position(self):
        # flies to first waypoint with unspecified velocity
        self.flight_manager.fly_to_position(1.0,1.0,FLIGHT_ALT)

    def __del__(self):
        self.flight_manager = None

# Main entry point of the script
if __name__ == "__main__":

    # creates array of waypoints (square flight path)
    waypoints = [
        (1.0, 1.0, FLIGHT_ALT),
        (1.0, -1.0, FLIGHT_ALT),
        (-1.0, -1.0, FLIGHT_ALT),
        (-1.0, 1.0, FLIGHT_ALT),
        (1.0, 1.0, FLIGHT_ALT),
    ]

    # creates array of velocities 1 per side
    velocities = [0.2, 0.1, 0.05, 0.025]

    print('Initializing Speed Commander')
    # Initialize the SpeedCommander
    demo = SpeedCommander(waypoints=waypoints,velocities=velocities)

    # waits until controller is switched to offboard mode
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
    # flies to starting position (no specified velocity)
    demo.go_to_first_position()
    print('Flying to starting position')

    # loop for each new waypoint (range - 1 because drone already flew to first waypoint)
    for id, i in enumerate(range(len(waypoints)-1)):
        print(f'Going to waypoint #{id}')
        demo.fly_to_next_waypoint()
        rospy.sleep(15)
    
    print('finished program. Exiting')
    rospy.signal_shutdown()
    exit(1)