import rospy
from connorscode.msg import AiDetection

def callback(data):
    if data.class_id == 605:
        rospy.loginfo("Frame: %d X: %d Y: %d", data.frame_id, data.x_min, data.y_min)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/tflite_data', AiDetection, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    rospy.spin()