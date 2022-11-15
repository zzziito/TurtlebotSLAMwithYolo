#!/usr/bin/env python3

import rospy 
import message_filters  
from std_msgs.msg import String 

def callback(data_odom, data_image):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s ,%s", data_odom.data, data_image.data)  


def listener():
    rospy.init_node('listener', anonymous=True) 

    sub_odom = message_filters.Subscriber("odom", String)
    sub_image = message_filters.Subscriber("image", String)

    sync_listener = message_filters.ApproximateTimeSynchronizer([sub_odom, sub_image], 10, 1, allow_headerless=True)

    sync_listener.registerCallback(callback)

    rospy.spin()  


if __name__ == '__main__':
    listener()

