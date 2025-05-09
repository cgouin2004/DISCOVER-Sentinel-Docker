


class WayPoint:
    def __int__(self, lat, lon, alt, vel):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vel = vel



class VehicleMission:
    def __int__(self, waypoints):
        self.waypoints = waypoints



    def reached_waypoint(self, waypoint):
        #TODO
        pass
