#!/usr/bin/env python

import rospy
import curses
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped


class DroneInfo:
    def __init__(self, stdscr):
        # Set up the screen with curses
        self.stdscr = stdscr
        curses.curs_set(0)

        # Initializing the node
        rospy.init_node('sg_drone_info_display_node', anonymous=True)

        # Subscribers
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_callback)

        self.current_mode = ''
        self.current_position = None
        self.sent_position = None
        self.current_velocity = None

        # Display values in a loop
        rate = rospy.Rate(2)  # 2Hz
        while not rospy.is_shutdown():
            self.display_info()
            rate.sleep()

    def state_callback(self, data):
        self.current_mode = data.mode

    def pose_callback(self, data):
        self.current_position = data

    def velocity_callback(self, data):
        self.current_velocity = data

    def display_info(self):
        #TODO update the display to a more beautiful decoration.
        self.stdscr.clear()

        self.stdscr.addstr(0, 0, "------ Drone Info ------")

        if self.current_position:
            self.stdscr.addstr(2, 0,
                               "Current Position (X, Y, Z): ({}, {}, {})".format(self.current_position.pose.position.x,
                                                                                 self.current_position.pose.position.y,
                                                                                 self.current_position.pose.position.z))
        else:
            self.stdscr.addstr(2, 0, "Current Position: Data not yet available")

        if self.sent_position:
            self.stdscr.addstr(3, 0, "Sent Position (X, Y, Z): ({}, {}, {})".format(self.sent_position.pose.position.x,
                                                                                    self.sent_position.pose.position.y,
                                                                                    self.sent_position.pose.position.z))
        else:
            self.stdscr.addstr(3, 0, "Sent Position: Data not yet available")

        if self.current_velocity:
            self.stdscr.addstr(4, 0, "Velocity (X, Y, Z): ({}, {}, {})".format(self.current_velocity.twist.linear.x,
                                                                               self.current_velocity.twist.linear.y,
                                                                               self.current_velocity.twist.linear.z))
        else:
            self.stdscr.addstr(4, 0, "Velocity: Data not yet available")

        self.stdscr.addstr(5, 0, "Current Mode: {}".format(self.current_mode))

        self.stdscr.refresh()


if __name__ == '__main__':
    try:
        curses.wrapper(DroneInfo)
    except rospy.ROSInterruptException:
        pass
