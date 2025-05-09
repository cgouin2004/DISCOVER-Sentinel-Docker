
# !/usr/bin/env python
from SGFlightKitCore import  FlightManager

import rospy
import threading
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

import numpy as np

FLIGHT_ALT = 1.3
TORSO_AREA = 20000

class GestureCommander:
    def __init__(self):


        self.flight_manager = FlightManager()
        self.pose_sub = rospy.Subscriber("hands_status", String, self.gesture_callback)

        self.torso_area_buffer = ["none"] * 3 # <- this value determines buffer size
        self.human_location_buffer = ["none"] * 3 # <- this value determines buffer size
        self.current_position = 0.0

        self.flight_manager.wait(2)

        print('Sending setpoint 0,0,FLIGHT_ALT before offboard mode is enabled')    
        self.flight_manager.fly_to_position(0.0, 0.0, FLIGHT_ALT)

        while (not rospy.is_shutdown() and not self.flight_manager.is_vehicle_mode_offboard()):
            print("Offboard is not enabled! Waiting for Offboard Mode...")
            self.flight_manager.wait(1)
            pass

        if rospy.is_shutdown():
            return

        print(f"Offboard is enabled! {self.flight_manager.is_vehicle_mode_offboard()}")
        '''
        print(f'\tWaiting 5 seconds before routine...')

        self.flight_manager.wait(5)

        print("Flying to 0,0,2. Waiting 5...")
        self.flight_manager.fly_to_position(0, 0, 2)
        self.flight_manager.wait(5)

        print("Flying to -1,0,2. Waiting 5...")
        self.flight_manager.fly_to_position(-1, 0, 2)
        self.flight_manager.wait(5)

        print("Flying to 0,0,2. Waiting 5...")
        self.flight_manager.fly_to_position(0, 0, 2)
        self.flight_manager.wait(5)

        print("Flying to 1,0,2. Waiting 5...")
        self.flight_manager.fly_to_position(1, 0, 2)
        self.flight_manager.wait(5)

        print("Flying to 0,0,2. Waiting 5...")
        self.flight_manager.fly_to_position(0, 0, 2)
        self.flight_manager.wait(5)

        print("Flight routine finished!")
        '''



        # self.flight_manager.offboard_mode_enable()

        # self.flight_manager.wait(1)


        # self.flight_manager.arm()



    def gesture_callback(self, data):
        #return
        torso_area, human_loc = str(data.data).split('-')
        torso_area = int(torso_area)

        if torso_area < 8000:
            print('ERROR, Human detection is too small')
        elif torso_area > 60000: 
            print("ERROR: Human detection too large")



        self.torso_area_buffer.pop()
        self.torso_area_buffer.insert(0, torso_area)
        self.human_location_buffer.pop()
        self.human_location_buffer.insert(0, human_loc)

        cur_x = self.flight_manager.current_local_loc.pose.position.x
        cur_y = self.flight_manager.current_local_loc.pose.position.y
        cur_z = self.flight_manager.current_local_loc.pose.position.z


        x_goal = cur_x
        y_goal = cur_y
        z_goal = cur_z

        if all(i == "right" for i in self.human_location_buffer):
            x_goal = cur_x + 0.15
        elif all(i == "left" for i in self.human_location_buffer):
            x_goal = cur_x - 0.15
        elif all(i == "center" for i in self.human_location_buffer):
            x_goal = cur_x
            

        if np.average(self.torso_area_buffer) > TORSO_AREA*1.4:
            y_goal = cur_y - 0.2
        elif np.average(self.torso_area_buffer) > TORSO_AREA*1.1:
            y_goal = cur_y - 0.1
        elif np.average(self.torso_area_buffer) < TORSO_AREA*0.6:
            y_goal = cur_y + 0.2
        elif np.average(self.torso_area_buffer) < TORSO_AREA*0.9:
            y_goal = cur_y + 0.1
        else:
            y_goal = cur_y

        self.flight_manager.fly_to_position(x_goal, y_goal, FLIGHT_ALT)
        print(f'current target: x:{x_goal}, y:{y_goal}, z:{FLIGHT_ALT}')

    def __del__(self):
        self.flight_manager = None



def dev():
    pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    message = PositionTarget()
    message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

    # Set type mask to ignore everything except for position
    message.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                        PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                        PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE

    # Set your desired position
    message.position.x = 0.0  # x position in meters, for example
    message.position.y = 0.0
    message.position.z = 1.0

    rate = rospy.Rate(10)  # 10Hz or whatever rate is suitable
    while not rospy.is_shutdown():
        pub.publish(message)
        rate.sleep()



if __name__ == "__main__":

    demo = GestureCommander()

    if not rospy.is_shutdown():
        rospy.spin()
