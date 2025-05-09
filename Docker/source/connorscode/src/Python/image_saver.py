#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Save the OpenCV image as a .jpg file
    image_path = "image.jpg"
    cv2.imwrite(image_path, cv_image)
    rospy.loginfo("Image saved to %s", image_path)

def main():
    rospy.init_node('image_saver', anonymous=True)
    rospy.Subscriber('/hires_small_color', Image, image_callback)
    rospy.loginfo("Subscribed to /hires_small_color topic")

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
