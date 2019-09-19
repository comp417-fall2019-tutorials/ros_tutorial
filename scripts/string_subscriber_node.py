#!/usr/bin/env python2

import rospy
from std_msgs.msg import String


def on_msg_received(msg):
    rospy.loginfo("Subscriber got %s" % msg.data)


if __name__ == '__main__':

    rospy.init_node('string_subscriber', anonymous=True)
    rospy.loginfo("String Subscriber node starting")
    msg_pub = rospy.Subscriber('/msg', String, on_msg_received)

    while not rospy.is_shutdown():
        rospy.spin()

