#!/usr/bin/env python3
import rospy
from demo_yolo.msg import Data
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " Name:", data.data)
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Data, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
