
# !/usr/bin/env python
from SGFlightKitCore import  FlightManager

import rospy
import threading
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

FLIGHT_ALT = 1.3

class GestureCommander:
    def __init__(self):


        self.flight_manager = FlightManager()
        self.pose_sub = rospy.Subscriber("hands_status", String, self.gesture_callback)

        self.human_hand_status_buffer = ["none"] * 3 # <- this value determines buffer size
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


    def move_left(self, yaw):
        print(f"Moving to Left Setpoint, yaw target = {yaw}")
        target_x = -1.0
        self.flight_manager.fly_to_position(target_x, 0.0, FLIGHT_ALT, yaw)
        self.current_position = target_x

    def move_right(self, yaw):
        print(f"Moving to Right Setpoint, yaw target = {yaw}")
        target_x = 1.0
        self.flight_manager.fly_to_position(target_x, 0.0, FLIGHT_ALT, yaw)
        self.current_position = target_x

    def return_to_center(self, yaw):
        print(f"Moving to Center Setpoint, yaw target = {yaw}")
        target_x = 0.0
        self.flight_manager.fly_to_position(target_x, 0.0, FLIGHT_ALT, yaw)
        self.current_position = target_x

    def hover(self, yaw):
        print(f"Hovering at current setpoint, yaw target = {yaw}")
        target_x = self.current_position
        self.flight_manager.fly_to_position(target_x, 0.0, FLIGHT_ALT, yaw)
        self.current_position = target_x

    def gesture_callback(self, data):
        #return
        hands_status, human_loc = str(data.data).split('-')

        self.human_hand_status_buffer.pop()
        self.human_hand_status_buffer.insert(0, hands_status)
        self.human_location_buffer.pop()
        self.human_location_buffer.insert(0, human_loc)

        yaw_goal = self.flight_manager.current_orientation.get('yaw')

        if all(i == "right" for i in self.human_location_buffer):
            yaw_goal -= 8 *(3.14159265 / 180.0) # 8 degree change
        elif all(i == "left" for i in self.human_location_buffer):
            yaw_goal += 8 *(3.14159265 / 180.0) # 8 degree change
        elif all(i == "center" for i in self.human_location_buffer):
            pass

        if all(i == "right" for i in self.human_hand_status_buffer):
            self.move_left(yaw_goal)
        elif all(i == "left" for i in self.human_hand_status_buffer):
            self.move_right(yaw_goal)
        elif all(i == "both" for i in self.human_hand_status_buffer):
            self.return_to_center(yaw_goal)
        elif all(i == "none" for i in self.human_hand_status_buffer):
            self.return_to_center(yaw_goal)
        # if last 3 human pose estimates are inconsistent, hover
        else:
            self.hover(yaw_goal)

        print(hands_status)
        print(human_loc)
        print(self.human_hand_status_buffer)
        #print(f'current orientation: {self.flight_manager.current_orientation}')
        #print(f'current yaw: {self.flight_manager.current_orientation.get("yaw", None)*180/3.1415926535:.6} degrees')
        #print(f'current yaw: {self.flight_manager.current_orientation.get("yaw", None):.6} radians')

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
