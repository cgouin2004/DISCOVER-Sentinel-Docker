#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import State, PositionTarget, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandBoolRequest, SetModeRequest
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import threading
from SGGeometry import SpatialPosition
import math
import json

# Services
TAKEOFF_SERVICE_URI = '/mavros/cmd/takeoff_local'
LAND_SERVICE_URI    = '/mavros/cmd/land_local'
ARM_SERVICE_URI     = '/mavros/cmd/arming'
SETMODE_SERVICE_URI = '/mavros/set_mode'


# Topics to subscribe
CURRENT_STATE_TOPIC_NAME    = '/mavros/state'
GLOBAL_POSITION_TOPIC_NAME  = '/mavros/global_position/global'
LOCAL_POSITION_TOPIC_NAME   = '/mavros/local_position/pose'
IMU_TOPIC_NAME              = '/mavros/imu/data'
COMPASS_TOPIC_NAME          = '/mavros/global_position/compass_hdg'

# Topics to publish
SET_VELOCITY_TOPIC_NAME = '/mavros/setpoint_velocity/cmd_vel'
SET_LOCAL_POSITION_TOPIC_NAME = '/mavros/setpoint_raw/local'

OFFBOARD_MODE_VALUE = "OFFBOARD"



def log(string):
    # rospy.loginfo(string)
    print(string)


class Orientation:
    """Container for roll, pitch, yaw in both radians and degrees."""
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

class _ROSServices:
    def __init__ (self, service_name, type, wait = True):
        
        if wait:
            rospy.wait_for_service(service_name)
        else:
            pass
        
        self.service = rospy.ServiceProxy(service_name, type)

    def command(data):
        pass





class ArmService(_ROSServices):
    def __init__(self):
        super().__init__(ARM_SERVICE_URI, CommandBool)


    def arm(self) -> bool:
        if FlightManager.util_is_vehicle_armed():
            log("Vehicle is already armed")
            return True
        request = CommandBoolRequest()
        request.value = True
        if (FlightManager.util_is_vehicle_mode_offboard()):
            response = self.service.call(request)
            if response.success:
                log("Vehicle is armed")
                return True
            else:
                log("Vehicle cannot be armed!")
                return False
        else:
            log("Vehicle is not ready to be armed, check connection or mode")
            return False

    def disarm(self) -> bool:
        #TODO
        pass



class SetModeService(_ROSServices):
    def __init__(self):
        super().__init__(SETMODE_SERVICE_URI, SetMode)


    def set_mode_to_offboard(self):

        if FlightManager.util_is_vehicle_mode_offboard():
            log(f"mode is already {OFFBOARD_MODE_VALUE}")
            return True
        
        reqeust = SetModeRequest()
        reqeust.custom_mode = OFFBOARD_MODE_VALUE

        if FlightManager.util_is_vehicle_connected:
            if self.service.call(reqeust).mode_sent:
                log(f"Mode is set to {OFFBOARD_MODE_VALUE}")
                return True
            else:
                log(f"Mode is not set to {OFFBOARD_MODE_VALUE}")
        else:
            log("Vehicle is not connected!")


class TakeOffService(_ROSServices):
    def __init__(self):
        super().__init__(TAKEOFF_SERVICE_URI, CommandTOL)  # Note: Takeoff typically uses CommandTOL service.

    def take_off(self, altitude=2.5) -> bool:
        if not FlightManager.util_is_vehicle_armed():
            log("Vehicle is not armed. Arming first...")
            if not FlightManager.util_is_vehicle_armed():  # Ensure the vehicle is armed.
                return False
            rospy.sleep(2)  # Small delay to ensure arming is complete.

        takeoff_request = CommandTOL()
        takeoff_request.altitude = altitude  # Set desired altitude.
        response = self.service.call(takeoff_request)

        if response.success:
            log(f"Take off command accepted. Rising to {altitude} meters.")
            return True
        else:
            log("Take off command was rejected!")
            return False


class LandService(_ROSServices):
    def __init__(self):
        super().__init__(LAND_SERVICE_URI, CommandTOL)  # Note: Land also typically uses CommandTOL service.

    def land(self, latitude=None, longitude=None, altitude=None) -> bool:
        land_request = CommandTOL()

        # If specific landing coords are provided, use them.
        if latitude is not None and longitude is not None:
            land_request.latitude = latitude
            land_request.longitude = longitude

        if altitude is not None:
            land_request.altitude = altitude

        response = self.service.call(land_request)

        if response.success:
            log("Land command accepted.")
            return True
        else:
            log("Land command was rejected!")
            return False


class _ROSPublisher:
    def __init__(self, *args, **kwargs) -> None:
        self.publisher = rospy.Publisher(*args, **kwargs)

    def publish(self, data):
        self.publisher.publish(data)



class LocalPosePublisher(_ROSPublisher):

    def __init__(self) -> None:
        super().__init__(name=SET_LOCAL_POSITION_TOPIC_NAME, data_class=PositionTarget, queue_size=10)
        self.ros_rate = rospy.Rate(10)
        self.pose = None
        self.async_pose_publisher_loop = threading.Thread(target=self._publish_pose)
        self.async_pose_publisher_loop.start()

    def _publish_pose(self):
        if threading.current_thread() == threading.main_thread():
            raise RuntimeError("This function cannot be called from the main thread!")
        while True:
            if FlightManager.util_is_vehicle_connected() and self.pose is not None:
                self.publish(self.pose)
                #print("set point sent ", self.pose)
                # log("setpoint published")
            self.ros_rate.sleep()

    def set_pose(self, x=None, y=None, z=None, velx=None, vely=None, velz=None, yaw=None):
        pose = PositionTarget()

        pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        pose.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE

        if (x is None): pose.type_mask += PositionTarget.IGNORE_PX
        else: pose.position.x = x

        if (y is None): pose.type_mask += PositionTarget.IGNORE_PY
        else: pose.position.y = y

        if (z is None): pose.type_mask += PositionTarget.IGNORE_PZ
        else: pose.position.z = z

        if (velx is None): pose.type_mask += PositionTarget.IGNORE_VX
        else: pose.velocity.x = velx

        if (vely is None): pose.type_mask += PositionTarget.IGNORE_VY
        else: pose.velocity.y = vely

        if (velz is None): pose.type_mask += PositionTarget.IGNORE_VZ
        else: pose.velocity.z = velz

        if (yaw is None): pose.type_mask += PositionTarget.IGNORE_YAW
        else: pose.yaw = yaw

        
        # if None in [velx, vely, velz] and yaw is None:
        #     pose.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
        #                 + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ
        #                 + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE)
        #     pose.position.x = x
        #     pose.position.y = y
        #     pose.position.z = z
        # elif None in [velx, vely, velz]:
        #     pose.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
        #                 + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_YAW_RATE)
        #     pose.yaw = yaw
        #     pose.position.x = x
        #     pose.position.y = y
        #     pose.position.z = z
        # elif yaw is None:
        #     pose.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
        #                 + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE)
        #     pose.velocity.x = velx
        #     pose.velocity.y = vely
        #     pose.velocity.z = velz
        # else:
        #     pose.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE)
        #     pose.yaw = yaw
        #     pose.velocity.x = velx
        #     pose.velocity.y = vely
        #     pose.velocity.z = velz
        
        self.pose = pose


    def kill_loop(self):
        self.pose = None
        log("****pose is none now****")





class VelocityPublisher(_ROSPublisher):
    def __init__(self):
        super().__init__(name=SET_VELOCITY_TOPIC_NAME, data_class=TwistStamped, queue_size=10)
        self.ros_rate = rospy.Rate(20)


    def set_velocity(self, z):
        velocity = TwistStamped()
        velocity.twist.angular.z = 1

        while True:
            # log('velocity is being called!')
            self.publish(velocity)
            self.ros_rate.sleep()




class _FlightCore:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(_FlightCore, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __del__(self):
        self.local_pose_publisher = None
        self.velocity_publisher = None

    def __init__(self) -> None:
        # Node name for identification
        self.node_name = "sg_flight_node"
        
        # Variables to store drone's position and orientation
        self.start_local_loc = PoseStamped()
        self.start_global_loc = NavSatFix()
        self.current_local_loc = PoseStamped()
        self.current_global_loc = NavSatFix()
        self.current_orientation = Quaternion()
        self.current_global_imu = Imu()
        self.current_global_orientation = Orientation()
       
        # Drone State 
        self.current_state = State()

        # Flag for indoor missions
        self.indoor_mission = False
        
        # Initialize the ROS node
        rospy.init_node(self.node_name, anonymous=True)

        print("Node has been initialized")

        self.ros_rate = rospy.Rate(20)


        #Register for mavros services
        # self.takeoff_commander = TakeOffService(False)
        #self.arm_commander = ArmService()
        #self.setMode_commander = SetModeService()
        #self.land_commander = LandService()
        #self.takeoff_commander = TakeOffService()


        #print("Services are added!")

        #Regsiter for publishers
        self.local_pose_publisher = LocalPosePublisher()
        self.velocity_publisher = VelocityPublisher()

        print("Publishers are added!")

        # Set up ROS Subscribers for drone's feedback
        rospy.Subscriber(CURRENT_STATE_TOPIC_NAME, State, self._current_state_callback)
        rospy.Subscriber(GLOBAL_POSITION_TOPIC_NAME, NavSatFix, self._current_global_position_callback)
        rospy.Subscriber(LOCAL_POSITION_TOPIC_NAME, PoseStamped, self._current_local_position_callback)
        rospy.Subscriber(IMU_TOPIC_NAME, Imu, self._current_imu_orientation_callback)
        rospy.Subscriber(COMPASS_TOPIC_NAME, Float64, self._current_yaw_callback)

        print("Subscribers are added!")

    def is_vehicle_connected(self) -> bool:
        return (not rospy.is_shutdown() and self.current_state.connected)
    

    def is_vehicle_mode_offboard(self) -> bool:
        return (self.is_vehicle_connected() and self.current_state.mode == OFFBOARD_MODE_VALUE)
    

    def is_vehicle_armed(self) -> bool:
        return self.current_state.armed
    

    def offboard_mode_enable(self) -> bool:
        return self.setMode_commander.set_mode_to_offboard()


    def wait_for_device_get_connected(self):
        while (not self.is_vehicle_connected()):
            self.ros_rate.sleep()


    def fly_to_position(self, x=None, y=None, z=None, velx=None, vely=None, velz=None, yaw=None):
        self.local_pose_publisher.set_pose(x=x, y=y, z=z, velx=velx, vely=vely, velz=velz, yaw=yaw)
        # self._update_pose_list(SpatialPosition(x, y, z))

    def spin_z(self, speed):
        self.velocity_publisher.set_velocity(speed)



    def stop_moving(self):
        #TODO
        pass


    def wait(self, second=1):
        rospy.sleep(second)

    @classmethod
    def util_is_vehicle_connected(cls) -> bool:
        if cls._instance is not None:
            return cls._instance.is_vehicle_connected()
        else:
            return False
        
    
    @classmethod 
    def util_is_vehicle_mode_offboard(cls) -> bool:
        if cls._instance is not None:
            return cls._instance.is_vehicle_mode_offboard()
        else:
            return False
        
    
    @classmethod
    def util_is_vehicle_armed(cls) -> bool:
        if cls._instance is not None:
            return cls._instance.is_vehicle_armed()
        else:
            return False
        

    def _current_state_callback(self, data):
        """Callback for updates on the drone's state. 
        It can be extended to handle various state-related operations.
        """
        self.current_state = data
        
    def _current_imu_orientation_callback(self, msg: Imu):
        """Update orientation from IMU's fused quaternion."""
        # Extract quaternion
        self.current_global_imu = msg
        q = msg.orientation
        q_list = [q.x, q.y, q.z, q.w]
        # Convert to roll, pitch, yaw
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        # Store radians
        self.current_global_orientation.roll  = roll
        self.current_global_orientation.pitch = pitch
        # Store degrees wrapped [0,360)
        self.current_global_orientation.roll_deg  = (math.degrees(roll)  + 180.0) % 360.0 - 180.0
        self.current_global_orientation.pitch_deg = (math.degrees(pitch) + 180.0) % 360.0 - 180.0

    def _current_yaw_callback(self, msg: Float64):
        self.current_global_orientation.yaw_deg = (msg.data+180.0)%360.0-180.0
        self.current_global_orientation.yaw = math.radians(msg.data)

    def _current_global_position_callback(self, data):
        """Callback to update the drone's global position."""
        self.current_global_loc = data

    def _current_local_position_callback(self, data):
        """Callback to update the drone's local position and orientation."""
        self.current_local_loc = data
        orientation = data.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_orientation = {'roll': roll, 'pitch': pitch, 'yaw': yaw}



class FlightManager(_FlightCore):
    def __init__(self):
        super().__init__()
        self.sent_pose_list = []


    def _update_pose_list(self, new_pose):
        if (len(self.sent_pose_list) == 0):
            self.sent_pose_list.append(new_pose)
        else:
            last_pose = self.sent_pose_list[-1]
            if last_pose != new_pose:
                self.sent_pose_list.append(new_pose)

    def fly_to_position(self, x=None, y=None, z=None, velx=None, vely=None, velz=None, yaw=None):
        super().fly_to_position(x=x, y=y, z=z, velx=velx, vely=vely, velz=velz, yaw=yaw)
        self._update_pose_list(SpatialPosition(x, y, z))

    def load_waypoints(self, filename='mission.json', missions_dir='/home/ros/ros_ws/src/connorscode/missions'):
        # make sure there's exactly one slash between dir and file
        fullpath = missions_dir.rstrip('/') + '/' + filename

        with open(fullpath, 'r') as f:
            data = json.load(f)

        waypoints = []
        for record in data:
            waypoints.append({
                "lat":         record.get("lat"),
                "long":        record.get("long"),
                "alt":         record.get("alt"),
                "flight_type": record.get("flight_type"),
                "speed":       record.get("speed"),
                "delay":       record.get("delay"),
                "tolerance":   record.get("tolerance")
            })
        return waypoints

    def distance_and_yaw(self, lat1, long1, lat2, long2):
        """
        Compute the great-circle distance and yaw from a current point to a target point.

        Parameters
        ----------
        lat1, lon1 : float
            Current latitude and longitude in decimal degrees.
        lat2, lon2 : float
            Target latitude and longitude in decimal degrees.

        Returns
        -------
        distance : float
            Distance in meters.
        yaw : float
            Heading in degrees, where:
                0   = East
                90  = North
                180 = West
                270 = South
        """
        # Earth radius in meters
        R = 6378000  

        # convert degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(long1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(long2)

        # differences
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad

        # haversine formula
        a = (math.sin(delta_lat / 2) ** 2 +
                math.cos(lat1_rad) * math.cos(lat2_rad) *
                math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        # initial bearing (degrees clockwise from North)
        bearing = math.degrees(math.atan2(
            math.sin(delta_lon) * math.cos(lat2_rad),
            math.cos(lat1_rad) * math.sin(lat2_rad) -
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        ))
        bearing = (bearing + 360) % 360

        # convert so that East=0, North=90
        yaw = ((90 - bearing) + 180.0) % 360 - 180.0

        return distance, yaw
    
    def compute_velocity_components(self, lateral_dist, yaw, speed, height_diff = 0.0):
        """
        Compute velocity components along the X, Y, and Z axes to move 
        toward a target offset.

        Parameters
        ----------
        lateral_dist : float
            Horizontal (XY-plane) distance to the target (meters).
        
            Vertical distance to the target (meters). Positive = up.
        yaw : float
            Horizontal bearing toward the target, in degrees:
            0   = +X direction
            90  = +Y direction
            180 = -X direction
            270 = -Y direction
        speed : float
            Total speed magnitude (m/s).
        height_diff : float (default: 0.0)

        Returns
        -------
        velx : float
            Velocity along the X axis (m/s).
        vely : float
            Velocity along the Y axis (m/s).
        velz : float
            Velocity along the Z axis (m/s). Positive = up.
        """
        # total straight-line distance
        dist3d = math.hypot(lateral_dist, height_diff)
        if dist3d == 0:
            return 0.0, 0.0, 0.0

        # split speed into horizontal and vertical components
        horiz_speed = speed * (lateral_dist / dist3d)
        vert_speed  = speed * (height_diff   / dist3d)

        # decompose horizontal into X and Y
        yaw = math.radians(yaw)
        velx = horiz_speed * math.cos(yaw)
        vely = horiz_speed * math.sin(yaw)

        # Z component
        velz = vert_speed

        return velx, vely, velz
    
    def fly_simple(self, lateral_dist, yaw, speed, height_diff = 0.0):
        yaw = self.current_orientation.get('yaw')*180/math.pi + yaw
        (velx, vely, velz) = self.compute_velocity_components(lateral_dist, yaw, speed, height_diff)
        dist3d = math.hypot(lateral_dist, height_diff)
        yaw_rad = math.radians(yaw)
        self.fly_to_position(velx=velx, vely=vely, velz=velz, yaw = yaw_rad)
        rospy.sleep(dist3d/speed)
        self.fly_to_position(x=self.current_local_loc.pose.position.x, y=self.current_local_loc.pose.position.y, z=self.current_local_loc.pose.position.z)
    
    
    def fly_gps_velocity(self, lat, long, speed, dest_rel_alt, delay = 0.0, tolerance = 2.0):
        (distance, yaw) = self.distance_and_yaw(lat1=self.current_global_loc.latitude, long1=self.current_global_loc.longitude, lat2=lat, long2=long)
        while (distance > tolerance):
            if (distance < 3*tolerance and speed > 2.0 and delay != 0.0):
                new_speed = 2.0
            else: 
                new_speed = speed
            if (distance < 2*tolerance and speed > 1.0 and delay != 0.0):
                new_speed = 1.0
            else: 
                new_speed = speed
            z=dest_rel_alt
            (velx, vely, velz) = self.compute_velocity_components(lateral_dist=distance, yaw=yaw, speed=new_speed, height_diff=0.0)
            yaw=math.radians(yaw)
            self.fly_to_position(velx=velx, vely=vely, z=z, yaw=yaw)
            rospy.sleep(0.1)
            (distance, yaw) = self.distance_and_yaw(lat1=self.current_global_loc.latitude, long1=self.current_global_loc.longitude, lat2=lat, long2=long)
        if (delay != 0.0):
            self.fly_to_position(velx=0.0, vely=0.0, z=z)
            rospy.sleep(0.5)
        self.fly_to_position(x=self.current_local_loc.pose.position.x, y=self.current_local_loc.pose.position.y, z=z)
        rospy.sleep(delay)

    def initialize_takeoff(self, speed = 1.0, time = 6.0, delay = 0.0):
        self.fly_to_position(velx=0.0, vely=0.0, velz=speed)
        self.wait(time)
        startx = self.current_local_loc.pose.position.x
        starty = self.current_local_loc.pose.position.y
        startz = self.current_local_loc.pose.position.z
        self.fly_to_position(x=startx, y=starty, z=startz)
        self.wait(delay)
        return startx, starty, startz
    
    def initialize_landing(self, startx = None, starty = None, startz = None):
        self.fly_to_position(x=startx, y=starty, z=startz)
        self.wait(10.0)
        self.fly_to_position(x=startx, y=starty, velz = -0.75)
        self.wait(20.0)

    def fly_mission(self, filename='mission.json', missions_dir='/home/ros/ros_ws/src/connorscode/missions'):
        waypoints = self.load_waypoints(filename=filename, missions_dir=missions_dir)
        startx = None
        starty = None
        startz = None
        for i, wp in enumerate(waypoints):
            if (wp['flight_type']==0):
                (startx, starty, startz) = self.initialize_takeoff(speed=wp['speed'], time=wp['alt'], delay=wp['delay'])
            elif (wp['flight_type']==1):
                self.fly_gps_velocity(lat=wp['lat'], long=wp['long'], speed=wp['speed'], dest_rel_alt= startz + wp['alt'], delay=wp['delay'], tolerance=wp['tolerance'])
            elif (wp['flight_type']==2):
                self.initialize_landing(startx=startx, starty=starty, startz=startz)
            else: print('Invalid Flight_Type')
            print(f'WAYPOINT #{i} REACHED')

        print('Mission Finished')

    def fly_to_position_velocity(self, x, y, z, speed):
        pose = PositionTarget()
        speed = abs(speed)
        distanced = 0.1 # distance at which position control takes over (in meters)
        num_increments = 10 # number of time increments
        distancex = x-self.current_local_loc.pose.position.x
        distancey = y-self.current_local_loc.pose.position.y
        distancez = z-self.current_local_loc.pose.position.z
        distancet = math.sqrt(distancex**2+distancey**2+distancez**2) # calculates total distance drone is projected to travel
        interval = distancet/speed/num_increments # time interval between distance checks
        while distancet > distanced:
            distancex = x-self.current_local_loc.pose.position.x
            distancey = y-self.current_local_loc.pose.position.y
            distancez = z-self.current_local_loc.pose.position.z
            distancet = math.sqrt(distancex**2+distancey**2+distancez**2) # calculates total distance drone is projected to travel
            velx=speed*(distancex/distancet) # calculates velocity components
            vely=speed*(distancey/distancet)
            velz=speed*(distancez/distancet)
            pose.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
                        + PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
                        + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE)
            pose.velocity.x = velx
            pose.velocity.y = vely
            pose.velocity.z = velz
            self.local_pose_publisher.pose=pose # this creates a waypoint to the drone
            if distancet > (interval*speed):
                rospy.sleep(interval)
            else:
                rospy.sleep(distancet/speed)
        self.fly_to_position(x, y, z)


    def take_off(self, altitude):
        """Command the drone to take off to a specified altitude."""
        self.takeoff_commander.take_off(altitude)

    def arm(self):
        """Arm the drone."""
        self.arm_commander.arm()

    def disarm(self):
        """Disarm the drone."""
        self.arm_commander.disarm()


    #TODO complete mission set up for the vehicle.
    def set_mission(self ):
        pass

if __name__ == "__main__":

    # Just a test to verify the package is working fine!
    flight = FlightManager()
    flight.wait_for_device_get_connected()

    flight.fly_to_position(0.0, 0.0, 0.0)
    print('test')

    rate = rospy.Rate(20)
    while True:
        flight.offboard_mode_enable()
        rate.sleep()
        if(flight.is_vehicle_mode_offboard()):
            break

    flight.wait(1)

    flight.arm()

    flight.wait(10)

    flight.local_pose_publisher.kill_loop()

    # flight.spin_z(1)





    # flight.spin_z(.1)

    flight.wait(200)
#
#     pass








    


