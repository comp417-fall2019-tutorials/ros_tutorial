#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist


def send_vel_command(throttle, steering):
    motor_command = Twist()
    motor_command.linear.x = throttle
    motor_command.angular.z = steering
    motor_command_pub.publish(motor_command)


if __name__ == '__main__':

    rospy.init_node('publisher_node', anonymous=True)
    motor_command_pub = rospy.Publisher('/cmd_vel', Twist)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        send_vel_command(1.0, 0.2)
        rate.sleep()
