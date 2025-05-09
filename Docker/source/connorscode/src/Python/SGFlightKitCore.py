#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandBoolRequest, SetModeRequest
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion
import threading
from SGGeometry import SpatialPosition
import math

# Services
TAKEOFF_SERVICE_URI = '/mavros/cmd/takeoff_local'
LAND_SERVICE_URI    = '/mavros/cmd/land_local'
ARM_SERVICE_URI     = '/mavros/cmd/arming'
SETMODE_SERVICE_URI = '/mavros/set_mode'


# Topics to subscribe
CURRENT_STATE_TOPIC_NAME    = '/mavros/state'
GLOBAL_POSITION_TOPIC_NAME  = '/mavros/global_position/global'
LOCAL_POSITION_TOPIC_NAME   = '/mavros/local_position/pose'

# Topics to publish
SET_VELOCITY_TOPIC_NAME = '/mavros/setpoint_velocity/cmd_vel'
SET_LOCAL_POSITION_TOPIC_NAME = '/mavros/setpoint_raw/local'

OFFBOARD_MODE_VALUE = "OFFBOARD"



def log(string):
    # rospy.loginfo(string)
    print(string)



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

    def set_pose(self, x, y, z, velx=None, vely=None, velz=None, yaw=None):
        pose = PositionTarget()
        pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        if None in [velx, vely, velz] and yaw is None:
            pose.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
                        + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ
                        + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE)
        elif None in [velx, vely, velz]:
            pose.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
                        + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_YAW_RATE)
            pose.yaw = yaw
        elif yaw is None:
            pose.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
                        + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE)
            pose.velocity.x = velx
            pose.velocity.y = vely
            pose.velocity.z = velz
        else:
            pose.type_mask = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE)
            pose.yaw = yaw
            pose.velocity.x = velx
            pose.velocity.y = vely
            pose.velocity.z = velz
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
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


    def fly_to_position(self, x, y, z, velx=None, vely=None, velz=None, yaw=None):
        if None in [velx, vely, velz] and yaw is None:
            self.local_pose_publisher.set_pose(x, y, z)
        elif None in [velx, vely, velz]:
            self.local_pose_publisher.set_pose(x, y, z, yaw=yaw)
        elif yaw is None:
            self.local_pose_publisher.set_pose(x, y, z, velx=velx, vely=vely, velz=velz)
        else:
            self.local_pose_publisher.set_pose(x, y, z, velx=velx, vely=vely, velz=velz, yaw=yaw)
        self._update_pose_list(SpatialPosition(x, y, z))


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

    def fly_to_position(self, x, y, z, velx=None, vely=None, velz=None, yaw=None):
        if None in [velx, vely, velz] and yaw is None:
            super().fly_to_position(x, y, z)
        elif None in [velx, vely, velz]:
            super().fly_to_position(x, y, z, yaw=yaw)
        elif yaw is None:
            super().fly_to_position(x, y, z, velx=velx, vely=vely, velz=velz)
        else:
            super().fly_to_position(x, y, z, velx=velx, vely=vely, velz=velz, yaw=yaw)
        self._update_pose_list(SpatialPosition(x, y, z))

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








    


