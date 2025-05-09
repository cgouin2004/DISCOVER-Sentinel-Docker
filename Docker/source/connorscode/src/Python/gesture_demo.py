
# !/usr/bin/env python
from SGFlightKitCore import  FlightManager

import rospy
import threading
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

FLIGHT_ALT = 1.7

class GestureCommander:
    def __init__(self):


        self.flight_manager = FlightManager()
        self.pose_sub = rospy.Subscriber("hands_status", String, self.gesture_callback)

        self.human_pose_buffer = ["none"] * 3
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


    def move_left(self):
        print("Moving to Left Setpoint")
        target_x = -1.0
        self.flight_manager.fly_to_position(target_x, 0.0, FLIGHT_ALT)
        self.current_position = target_x

    def move_right(self):
        print("Moving to Right Setpoint")
        target_x = 1.0
        self.flight_manager.fly_to_position(target_x, 0.0, FLIGHT_ALT)
        self.current_position = target_x

    def return_to_center(self):
        print("Moving to Center Setpoint")
        target_x = 0.0
        self.flight_manager.fly_to_position(target_x, 0.0, FLIGHT_ALT)
        self.current_position = target_x

    def hover(self):
        print("Hovering at current setpoint")
        target_x = self.current_position
        self.flight_manager.fly_to_position(target_x, 0.0, FLIGHT_ALT)
        self.current_position = target_x

    def gesture_callback(self, data):
        #return
        self.human_pose_buffer.pop()
        self.human_pose_buffer.insert(0, data.data)
        if all(i == "right" for i in self.human_pose_buffer):
            self.move_left()
        elif all(i == "left" for i in self.human_pose_buffer):
            self.move_right()
        elif all(i == "both" for i in self.human_pose_buffer):
            self.return_to_center()
        elif all(i == "none" for i in self.human_pose_buffer):
            self.return_to_center()
        # if last 5 human pose estimates are inconsistent, hover
        else:
            self.hover()

        #print(self.human_pose_buffer)

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
