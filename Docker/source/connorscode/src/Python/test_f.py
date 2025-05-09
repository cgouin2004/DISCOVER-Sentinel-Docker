#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandLong

def set_home_position(latitude, longitude, altitude):
    rospy.wait_for_service('/mavros/cmd/command')
    try:
        send_command = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        # MAV_CMD_DO_SET_HOME is the MAVLink command to set home position
        # Command ID is 179 for MAV_CMD_DO_SET_HOME
        # More details on MAV_CMD_DO_SET_HOME can be found in MAVLink specification

        response = send_command.call(
            broadcast=False,
            command=179,  # MAV_CMD_DO_SET_HOME
            confirmation=0,
            param1=1,  # Set current location as home. If you set param1=0, it will use the lat, long, and altitude provided.
            param5=latitude,
            param6=longitude,
            param7=altitude
        )

        if response.success:
            rospy.loginfo("Home position set successfully!")
        else:
            rospy.logwarn("Failed to set home position!")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('set_home_node')

    # Example coordinates, replace with desired values
    desired_latitude = 47.39804
    desired_longitude = 8.54594
    desired_altitude = 400.0

    set_home_position(desired_latitude, desired_longitude, desired_altitude)
