#!/usr/bin/env python3

import rospy  # 导入rospy包 ROS的python客户端
from std_msgs.msg import String  # 导入std_msgs.msg包的String消息类型


def talker():
    pub = rospy.Publisher('odom', String, queue_size=1) 
    rospy.init_node('odom_talker', anonymous=True)  
    rate = rospy.Rate(50) 

    while not rospy.is_shutdown():
        hello_str = "odom %s" % rospy.get_time()
        rospy.loginfo(hello_str) 
        pub.publish(hello_str)
        rate.sleep() 


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

