#!/usr/bin/env python
"""
Simple test script for SGVideo.VideoRecorder with hardcoded configuration constants.
"""
import SGFlightKitCore
import rospy
from SGVideo import VideoRecorder

# === Configuration Constants ===
TOPIC_NAME   = '/hires_small_color'  # image topic to record
DURATION     = 60.0         # seconds to record
OUTPUT_FILE  = '/home/ros/ros_ws/src/connorscode/video/test_output1.mp4'
FRAME_RATE   = 5.0         # frames per second

TOPIC_NAME2   = '/tracking'  # image topic to record
OUTPUT_FILE2  = '/home/ros/ros_ws/src/connorscode/video/test_output2.mp4'



def main():
    # Initialize ROS node
    rospy.init_node('test_video_recorder', anonymous=True)

    # Instantiate recorder with constants
    recorder = VideoRecorder(
        topic_name=TOPIC_NAME,
        frame_rate=FRAME_RATE,
        output_file=OUTPUT_FILE
    )

    recorder2 = VideoRecorder(
        topic_name=TOPIC_NAME2,
        frame_rate=FRAME_RATE,
        output_file=OUTPUT_FILE2
    )
    # Start recording
    recorder.start()
    recorder2.start()
    rospy.loginfo("Recording %s seconds from topic '%s' @ %.2f FPS to '%s'", 
                  DURATION, TOPIC_NAME, FRAME_RATE, OUTPUT_FILE)

    # Sleep for duration, letting recorder capture frames
    rospy.sleep(DURATION)

    # Stop and finalize video
    recorder.stop()
    recorder2.stop()
    rospy.loginfo("Recording complete.")


if __name__ == '__main__':
    main()
