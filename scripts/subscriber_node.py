import rospy
from geometry_msgs.msg import Twist


def on_cmd_callback(vel):
    rospy.loginfo("throttle: %.2f, steering: %.2f" % (vel.linear_acceleration.x, vel.angular_velocity.z))


if __name__ == '__main__':

    rospy.init_node('subscriber_node', anonymous=True)
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        r.sleep()
        rospy.spin()
