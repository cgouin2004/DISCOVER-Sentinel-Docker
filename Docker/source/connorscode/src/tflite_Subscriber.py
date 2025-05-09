
import rospy
from connorscode.msg import AiDetection

def callback(data):
    rospy.loginfo("Received Segment %d to %d: x1, y1: %d, %d and x2, y2: %d, %d with confidence %f", data.class_id/100, data.class_id, data.x_min, data.y_min, data.x_max, data.y_max, data.detection_confidence)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/tflite_data', AiDetection, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    rospy.spin()