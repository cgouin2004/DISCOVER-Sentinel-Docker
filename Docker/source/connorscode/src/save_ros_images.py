#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/tflite", Image, self.callback)
        self.video_writer = None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # if self.video_writer is None:
        #     fourcc = cv2.VideoWriter_fourcc(*'XVID')
        #     height, width, _ = cv_image.shape
        #     self.video_writer = cv2.VideoWriter('output.avi', fourcc, 20.0, (width, height))

        # self.video_writer.write(cv_image)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

        print("received!")
    def shutdown_hook(self):
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('image_saver_node', anonymous=True)
    img_saver = ImageSaver()
    rospy.on_shutdown(img_saver.shutdown_hook)
    rospy.spin()
