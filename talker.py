#!/usr/bin/env python
# 

"""
Just an exemple of a talker with ROS. No goal except to have the structure. -> send (talker) to channel
"""
__author__ =  'Jonathan Muller <jonathan.muller at epfl.ch>'
__version__=  '1.0'
__license__ = 'BSD'

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
