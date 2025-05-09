# Import necessary libraries
from SGFlightKitCore import FlightManager
import rospy
import math
from mavros_msgs.msg import PositionTarget

# Define the flight altitude constant
FLIGHT_ALT = 1

# Define the CubeCommander class
class CubeCommander:
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
        print('Waiting for offboard mode')
    def fly_to_next_waypoint(self):
        while (not rospy.is_shutdown() and not self.flight_manager.is_vehicle_mode_offboard()):
            self.flight_manager.wait(1)
            pass

        if rospy.is_shutdown():
            return
        
        # Get the next waypoint from the list
        waypoint = self.waypoints[self.current_waypoint_index]
        # Fly to the waypoint
        self.flight_manager.fly_to_position(x=waypoint[0],y=waypoint[1],z=waypoint[2],yaw=waypoint[3])
        x=waypoint[0]
        y=waypoint[1]
        z=waypoint[2]
        yaw=waypoint[3]
        print(f'x: {x} y: {y} z: {z} yaw: {yaw}')
        # Increment the current waypoint index
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    def __del__(self):
        self.flight_manager = None

# Main entry point of the script
if __name__ == "__main__":

    pi=math.pi

    waypoints = [
            (0.5, 0.5, FLIGHT_ALT, pi/4),
            (0.5, -0.5, FLIGHT_ALT, 3*pi/2),
            (-0.5, -0.5, FLIGHT_ALT, pi),
            (-0.5, 0.5, FLIGHT_ALT, pi/2),
            (0.5, 0.5, FLIGHT_ALT, 0.0),
            (0.5, 0.5, FLIGHT_ALT+1, 0.0),
            (0.5, -0.5, FLIGHT_ALT+1, 3*pi/2),
            (-0.5, -0.5, FLIGHT_ALT+1, pi),
            (-0.5, 0.5, FLIGHT_ALT+1, pi/2),
            (0.5, 0.5, FLIGHT_ALT+1, 0.0)
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
    startx = demo.flight_manager.current_local_loc.pose.position.x
    starty = demo.flight_manager.current_local_loc.pose.position.y
    startz = demo.flight_manager.current_local_loc.pose.position.z
    demo.flight_manager.fly_to_position(x=startx, y=starty, z=startz)
    print('Offboard Mode Detected: Waiting 5 seconds')
    demo.flight_manager.wait(5)
    print('Beginning sending waypoints!')
    for id, i in enumerate(range(len(waypoints))):
        print(f'Goint to waypoint #{id}')
        demo.fly_to_next_waypoint()
        rospy.sleep(4)
    demo.flight_manager.fly_to_position(x=startx, y=starty, z=startz)
    print('finished program. Exiting')
    rospy.signal_shutdown()
    exit(1)
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #     rospy.spin()
