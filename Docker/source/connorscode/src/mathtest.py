import math
FLIGHT_ALT=1
waypoints = [
(1.0, 1.0, FLIGHT_ALT),
(1.0, -1.0, FLIGHT_ALT),
(-1.0, -1.0, FLIGHT_ALT),
(-1.0, 1.0, FLIGHT_ALT),
(1.0, 1.0, FLIGHT_ALT),
]
velocities = [0.25,0.75,1.25,1.75]
current_waypoint_index=0
waypoint = waypoints[current_waypoint_index+1]
pwaypoint = waypoints[current_waypoint_index]
distancex = waypoint[0]-pwaypoint[0]
distancey = waypoint[1]-pwaypoint[1]
distancez = waypoint[2]-pwaypoint[2]
distancet = math.sqrt(distancex**2+distancey**2+distancez**2)
velx=velocities[current_waypoint_index]*(distancex/distancet)
vely=velocities[current_waypoint_index]*(distancey/distancet)
velz=velocities[current_waypoint_index]*(distancez/distancet)
print(velocities[current_waypoint_index])
print(distancet)
print(velx)
print(vely)
print(velz)