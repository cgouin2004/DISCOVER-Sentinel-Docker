#!/usr/bin/env python
"""
ROS node that subscribes to the /tracking image topic and records incoming frames into an MP4 video file, without relying on cv_bridge.

Usage:
  rosrun <your_package> tracking_to_video.py _output_file:=tracking.mp4 _frame_rate:=10.0

Parameters:
  ~output_file (string): Path to the output MP4 file (default: /home/ros/ros_ws/src/connorscode/video/tracking_output.mp4)
  ~frame_rate  (float) : Desired video frame rate in FPS (default: 10.0)
"""
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class VideoRecorder:
    def __init__(self):
        # Load parameters
        default_path = '/home/ros/ros_ws/src/connorscode/video/tracking_output.mp4'
        self.output_file = rospy.get_param('~output_file', default_path)
        self.frame_rate = rospy.get_param('~frame_rate', 10.0)

        # VideoWriter will be created on first write
        self.writer = None
        # Timestamp of last frame written
        self.last_write_time = None

        # Subscribe to the /tracking topic
        self.sub = rospy.Subscriber('/tracking', Image, self.callback)

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

    def callback(self, msg):
        # Convert message to BGR frame
        if msg.encoding == 'mono8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            frame = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        elif msg.encoding in ['rgb8', 'bgr8']:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else arr
        else:
            rospy.logerr("Unsupported image encoding: %s", msg.encoding)
            return

        # Initialize VideoWriter on first valid frame
        if self.writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.output_file, fourcc, self.frame_rate, (w, h))
            rospy.loginfo("VideoWriter initialized: %s (%dx%d) @ %.2f FPS", 
                          self.output_file, w, h, self.frame_rate)

        # Throttle frames to desired frame_rate
        now = rospy.Time.now()
        if self.last_write_time is None:
            write_frame = True
        else:
            elapsed = (now - self.last_write_time).to_sec()
            write_frame = elapsed >= (1.0 / self.frame_rate)

        if write_frame:
            self.writer.write(frame)
            self.last_write_time = now

    def shutdown(self):
        rospy.loginfo("Shutting down, releasing video writer...")
        try:
            self.sub.unregister()
        except Exception:
            pass
        if self.writer:
            self.writer.release()
            rospy.loginfo("Video saved: %s", self.output_file)

if __name__ == '__main__':
    rospy.init_node('tracking_to_video', anonymous=False)
    VideoRecorder()
    rospy.spin()
