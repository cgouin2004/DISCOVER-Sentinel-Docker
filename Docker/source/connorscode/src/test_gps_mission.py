# Import necessary libraries
from SGFlightKitCore import FlightManager
import rospy
from mavros_msgs.msg import PositionTarget
import json

# Define the flight altitude constant
FLIGHT_ALT = 1

class WaypointLoader:
    def load_waypoints(self):
        filename = "Missions/data.json"  # JSON file to extract data from
        with open(filename, "r") as f:
            data = json.load(f)

        waypoints = []
        # Process each record and extract its values into a dictionary (waypoint)
        for index, record in enumerate(data):
            # Create a waypoint dictionary from the record data
            waypoint = {
                "lat": record.get("lat"),
                "long": record.get("long"),
                "alt": record.get("alt"),
                "flight_type": record.get("flight_type"),
                "speed": record.get("speed"),
                "delay": record.get("delay"),
                "tolerance": record.get("tolerance")
            }
            # Append the waypoint to our list
            waypoints.append(waypoint)
            
            # Optional: Print out the extracted values for debugging
            # print(f"Waypoint {index + 1}:")
            # print(f"  Latitude:    {waypoint['lat']}")
            # print(f"  Longitude:   {waypoint['long']}")
            # print(f"  Altitude:    {waypoint['alt']}")
            # print(f"  Flight Type: {waypoint['flight_type']}")
            # print(f"  Speed:       {waypoint['speed']}")
            # print(f"  Delay:       {waypoint['delay']}\n")
        
        return waypoints
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

       
        # Define the waypoints for the cube flight path

        self.current_waypoint_index = 0

    def fly_to_next_waypoint(self):
        
        # Get the next waypoint from the list
        waypoint = self.waypoints[self.current_waypoint_index]
        # Fly to the waypoint
        # self.flight_manager.fly_to_position(*waypoint)
        if waypoint['flight_type'] == 0:
            print(f"Taking off to {waypoint['alt']} meters")
        elif waypoint['flight_type'] == 2:
            print("Landing Drone")
        else:
            print("Flying to:", waypoint)
        
        print(f"Waypoint Reached. Waiting {waypoint['delay']} seconds.")
        # Increment the current waypoint index
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    def __del__(self):
        self.flight_manager = None

# Main entry point of the script
if __name__ == "__main__":
    print("Loading Waypoints")
    loader = WaypointLoader()
    waypoints = loader.load_waypoints()
    print("Waypoints Loaded Successfully")

    demo = CubeCommander(waypoints=waypoints)
    print('Initializing Flight Commander')
    # Initialize the CubeCommander
    
    print('Offboard Mode Detected: Waiting 5 seconds')
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
