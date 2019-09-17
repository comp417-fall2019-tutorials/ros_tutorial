#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist


def on_cmd_callback(motor_command):
    throttle = motor_command.linear.x
    steering = motor_command.angular.z
    rospy.loginfo("throttle: %.2f, steering: %.2f" % (throttle, steering))


if __name__ == '__main__':

    rospy.init_node('subscriber_node', anonymous=True)
    r = rospy.Rate(30)

    rospy.Subscriber('/cmd_vel', Twist, on_cmd_callback)
    while not rospy.is_shutdown():
        r.sleep()
        rospy.spin()
