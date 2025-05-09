#!/usr/bin/env python
"""
SGVideo: Simple ROS VideoRecorder API

Usage:
  from SGVideo import VideoRecorder
  recorder = VideoRecorder()  # defaults: topic='/tracking', fps=10.0,
                              #         output='/home/ros/ros_ws/src/connorscode/video/tracking_output.mp4'
  recorder.start()
  # … let it run …
  recorder.stop()
"""
import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

class VideoRecorder:
    def __init__(self,
                 topic_name: str = '/tracking',
                 frame_rate: float = 10.0,
                 output_file: str = None):
        # absolute default output path
        default_out = '/home/ros/ros_ws/src/connorscode/video/tracking_output.mp4'

        self.topic_name   = topic_name
        self.frame_rate   = frame_rate
        self.output_file  = output_file or default_out
        self.writer       = None
        self.last_write   = None
        self.subscriber   = None
        self._stopped     = False

    def start(self):
        if not rospy.core.is_initialized():
            rospy.init_node('video_recorder_node', anonymous=True)

        rospy.loginfo(f"SGVideo: subscribing to Image on '{self.topic_name}'")
        print  (f"SGVideo: subscribing to Image on '{self.topic_name}'")
        self.subscriber = rospy.Subscriber(self.topic_name, Image, self._callback)
        rospy.loginfo(f"SGVideo: will record @ {self.frame_rate:.2f} FPS → {self.output_file}")
        print  (f"SGVideo: will record @ {self.frame_rate:.2f} FPS → {self.output_file}")

    def _callback(self, msg: Image):
        enc = msg.encoding.lower()

        # decode into BGR frame
        if enc in ('mono8', 'raw8'):
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            try:
                arr = arr.reshape((msg.height, msg.width))
            except ValueError:
                rospy.logerr(f"SGVideo: bad reshape for {enc} {(msg.width, msg.height)}")
                print  (f"SGVideo: bad reshape for {enc} {(msg.width, msg.height)}")
                return
            frame = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

        elif enc == 'rgb8':
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            try:
                arr = arr.reshape((msg.height, msg.width, 3))
            except ValueError:
                rospy.logerr(f"SGVideo: bad reshape for {enc} {(msg.width, msg.height, 3)}")
                print  (f"SGVideo: bad reshape for {enc} {(msg.width, msg.height, 3)}")
                return
            frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

        elif enc == 'bgr8':
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            try:
                frame = arr.reshape((msg.height, msg.width, 3))
            except ValueError:
                rospy.logerr(f"SGVideo: bad reshape for {enc} {(msg.width, msg.height, 3)}")
                print  (f"SGVideo: bad reshape for {enc} {(msg.width, msg.height, 3)}")
                return

        elif enc in ('yuv422', 'yuyv'):
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            try:
                arr = arr.reshape((msg.height, msg.width, 2))
            except ValueError:
                rospy.logerr(f"SGVideo: bad reshape for {enc} {(msg.width, msg.height, 2)}")
                print  (f"SGVideo: bad reshape for {enc} {(msg.width, msg.height, 2)}")
                return
            frame = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_UYVY)

        elif enc == 'nv12':
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            try:
                arr = arr.reshape((int(msg.height * 1.5), msg.width))
            except ValueError:
                rospy.logerr(f"SGVideo: bad reshape for {enc} {(msg.width, int(msg.height * 1.5))}")
                print  (f"SGVideo: bad reshape for {enc} {(msg.width, int(msg.height * 1.5))}")
                return
            frame = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_NV12)

        elif enc == 'nv21':
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            try:
                arr = arr.reshape((int(msg.height * 1.5), msg.width))
            except ValueError:
                rospy.logerr(f"SGVideo: bad reshape for {enc} {(msg.width, int(msg.height * 1.5))}")
                print  (f"SGVideo: bad reshape for {enc} {(msg.width, int(msg.height * 1.5))}")
                return
            frame = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_NV21)

        else:
            rospy.logerr(f"SGVideo: unsupported encoding '{msg.encoding}'")
            print  (f"SGVideo: unsupported encoding '{msg.encoding}'")
            return

        # open VideoWriter on first frame
        if self.writer is None:
            folder = os.path.dirname(self.output_file)
            if folder and not os.path.isdir(folder):
                os.makedirs(folder, exist_ok=True)

            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.output_file,
                                          fourcc,
                                          self.frame_rate,
                                          (w, h))
            msg = f"SGVideo: writer opened {w}x{h} @ {self.frame_rate:.2f} FPS → {self.output_file}"
            rospy.loginfo(msg)
            print(msg)

        # throttle to target FPS
        now = rospy.Time.now()
        if self.last_write is None or (now - self.last_write).to_sec() >= 1.0 / self.frame_rate:
            self.writer.write(frame)
            self.last_write = now

    def stop(self):
        if self._stopped:
            return
        self._stopped = True

        # unregister subscriber
        if self.subscriber:
            try:
                self.subscriber.unregister()
            except Exception:
                pass
            self.subscriber = None

        # release writer (if any)
        if self.writer:
            rospy.loginfo("SGVideo: stopping, releasing writer")
            print  ("SGVideo: stopping, releasing writer")
            self.writer.release()
            self.writer = None

            msg = f"SGVideo: saved video to {self.output_file}"
            rospy.loginfo(msg)
            print  (msg)
        else:
            rospy.logwarn("SGVideo: stop() called but no frames were written")
            print  ("SGVideo: stop() called but no frames were written")
