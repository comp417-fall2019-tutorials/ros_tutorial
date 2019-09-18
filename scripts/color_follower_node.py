#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


class ColorFollower:

    def __init__(self, color_to_follow, throttle, steering_correction_gain):
        self.robot_command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_subscriber = rospy.Subscriber('/camera/image_raw', Image, self.on_image_callback, queue_size=1)
        self.cv_bridge = CvBridge()
        self.color_to_follow = color_to_follow
        self.throttle = throttle
        self.steering_correction_gain = steering_correction_gain

    def publish_engine_command(self, throttle, steering):
        robot_command = Twist()
        robot_command.linear.x = throttle
        robot_command.angular.z = steering
        self.robot_command_pub.publish(robot_command)

    def on_image_callback(self, ros_img):
        cv_img = self.cv_bridge.imgmsg_to_cv2(ros_img, "bgr8")
        indices = np.where((cv_img[:, :, 0] == color_to_follow[0])
                           & (cv_img[:, :, 1] == color_to_follow[1])
                           & (cv_img[:, :, 2] == color_to_follow[2]))
        matching_cols = indices[1]
        if len(matching_cols) > 0:
            steering_error = self.steering_correction_gain*(cv_img.shape[1]/2 - matching_cols[0])
            steering = steering_error
            self.publish_engine_command(self.throttle, steering)
        else:
            self.publish_engine_command(0, 0)


if __name__ == '__main__':

    rospy.init_node('color_follower', anonymous=True)
    rospy.loginfo("Color follower node starting")

    color_to_follow_str = rospy.get_param("~color_to_follow", "255,0,0")
    throttle = rospy.get_param("~throttle", 0.6)
    steering_correction_gain = rospy.get_param("~steering_correction_gain", 0.01)

    color_to_follow = tuple([int(c) for c in color_to_follow_str.replace(' ', '').split(',')])
    ros_tutorial = ColorFollower(color_to_follow, throttle, steering_correction_gain)

    while not rospy.is_shutdown():
        rospy.spin()
