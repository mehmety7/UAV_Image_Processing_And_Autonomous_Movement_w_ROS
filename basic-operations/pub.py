#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def my_pub():
    pub = rospy.Publisher('my_topic', String, queue_size=10)
    rospy.init_node('my_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        my_pub()
    except rospy.ROSInterruptException:
        pass

