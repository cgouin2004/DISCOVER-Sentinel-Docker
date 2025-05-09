# Import necessary libraries
from SGFlightKitCore import FlightManager
import rospy
from mavros_msgs.msg import PositionTarget
import math

# Define the flight altitude constant
FLIGHT_ALT = 1
FOVD = 67.38 # degrees
FOVR = 1.176 # radians

# Define the FollowCommander class
class FollowCommander:
    def __init__(self):
        # Initialize the FlightManager
        self.flight_manager = FlightManager()
        # Wait for 2 seconds before sending the setpoint

        self.flight_manager.wait(2)

        self.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT)
        print("Initialize starting position (0,0,FLIGHT_ALT)")
       
        # Define the waypoints for the cube flight path

        self.current_waypoint_index = 0


    def __del__(self):
        self.flight_manager = None

    def obtain_torso_size (self):

        pass

    def obtain_torso_position (self):
        pass

# Main entry point of the script
if __name__ == "__main__":

    
    print('Initializing Cube Commander')
    # Initialize the FollowCommander
    demo = FollowCommander()
    interval = .1 # how long it allows for flight before checking for another change in position
    size_buffer = 10 # controls idle distance (higher = more precise)
    position_buffer = 10 # controls idle rotation (lower = more precise)
    maxd = .5 # maximum distance change allowed per cycle
    filtern = 5 # number of pictures to reduce bad detections
    position_center = [None]
    torso_total=0

    print('Waiting for offboard mode!')
    while (not rospy.is_shutdown() and not demo.flight_manager.is_vehicle_mode_offboard()):
        demo.flight_manager.wait(1)

    if rospy.is_shutdown():
        rospy.signal_shutdown()
        exit(1)
    print('Offboard Mode Detected: Waiting 5 seconds')
    demo.flight_manager.wait(5)
    print('Initializing starting distance')
    torso_start = [None] # captures size of torso at start (for reference)
    torso_size=torso_start
    print('Beginning following sequence!')
    while (1):
        check=0
        while (check<4):
            torso_total=0
            for i in enumerate(range(filtern)): # takes a specified (filtern) numbeber of frames and averages them without using abnormal measurements
                torso_size=demo.obtain_torso_size()
                if abs(torso_size-torso_last) < (size_buffer*torso_last):
                    torso_total+=torso_size
                    torso_last=torso_size
                    check+=1
        torso_size = torso_total/check
        torso_position = demo.obtain_torso_position()
        if (abs(torso_size-torso_start) > (torso_start/size_buffer) or abs(torso_position-position_center) > position_buffer): # only move if too close or too far / not pointing at target
            yaw = demo.flight_manager.current_orientation.get('yaw')
            x = demo.flight_manager.current_local_loc.pose.position.x
            y = demo.flight_manager.current_local_loc.pose.position.y
            if (torso_start-torso_size) < 0:
                ddistance=2**(torso_start-torso_size) # calculates how much the drone should move
            else:
                ddistance=(torso_start-torso_size)*(1/2)
            if ddistance > maxd:
                ddistance = maxd
            dyaw = [None] # calculates how much the drone should rotate
            dx = math.cos(yaw+dyaw)*ddistance # calculates needed change in x and y
            dy = math.sin(yaw+dyaw)*ddistance
            demo.flight_manager.fly_to_position(x=x+dx, y=y+dy, z=FLIGHT_ALT, yaw=yaw+dyaw) # flies toward target
            print(f'Going to waypoint x={x+dx} y={y+dy} z={FLIGHT_ALT} with yaw={(yaw+dyaw)/math.pi} pi')
        rospy.sleep(interval) # loops after set amount of time
            
    
    print('finished program. Exiting')
    rospy.signal_shutdown()
    exit(1)
    # Start the ROS event loop (if rospy not shutdown)
    # if not rospy.is_shutdown():
    #     rospy.spin()