#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float32MultiArray
import numpy as np
from collections import deque
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState


RECT_DIMS = np.array([
    [6.0, 2.0],
    [6.0, -2.0],
    [5.0, -2.0],
    [5.0, 2.0],
])

POINTS_PER_SEGMENT = 100
POS_TRACKING_LEN = 1000
POS_TRACKING_MIN_DELTA = 0.05

pos_tracking = {
    'red_ball': deque(maxlen=POS_TRACKIN_LEN),
    'blue_ball': deque(maxlen=POS_TRACKIN_LEN),
    'green_ball': deque(maxlen=POS_TRACKIN_LEN)
}


def publish_rect():

    msg = PointCloud()
    msg.points = []
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'odom'

    for p_0, p in zip(RECT_DIMS, np.vstack((RECT_DIMS[1:], (RECT_DIMS[:1])))):
        line_points = np.linspace(p_0, p, POINTS_PER_SEGMENT)
        for lp in line_points:
            msg.points.append(Point(lp[0], lp[1], 0))

    secret_publisher.publish(msg)


def publish_tracking(points, odom_tracker_key):

    msg = PointCloud()
    msg.points = []
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'odom'

    for p in points:
        msg.points.append(Point(p.x, p.y, 0))

    odom_out_publishers[odom_tracker_key].publish(msg)


def check_pos():

    for k, pos in pos_tracking.items():
        state = get_model_state(k, '')
        position = state.pose.position
        last_position = pos[-1] if len(pos) > 0 else Point(0, 0, 0)
        dist = math.sqrt((position.x-last_position.x)**2 + (position.y-last_position.y)**2)
        if dist > POS_TRACKING_MIN_DELTA:
            pos.append(position)
            publish_tracking(pos, k)


if __name__ == '__main__':

    rospy.init_node('secret_node', anonymous=True)
    rospy.loginfo("Secret node starting")

    secret_publisher = rospy.Publisher('/secret', PointCloud, queue_size=1)
    odom_out_publishers = {k: rospy.Publisher('/pos_%s' % k, PointCloud, queue_size=1) for k in pos_tracking.keys()}

    r = rospy.Rate(15)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    while not rospy.is_shutdown():
        try:
            publish_rect()
            check_pos()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass


