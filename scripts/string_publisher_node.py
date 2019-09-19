#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

if __name__ == '__main__':

    rospy.init_node('string_publisher', anonymous=True)
    message_to_send = rospy.get_param("~msg_to_publish", "Default Message")

    msg_pub = rospy.Publisher('/msg', String, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = String()
        msg.data = message_to_send
        rospy.loginfo("Publisher sending %s" % msg.data)
        msg_pub.publish(msg)
        rate.sleep()
