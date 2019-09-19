#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import rospy
from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Adjust command velocity on vehicle
w - x:          Increase / Decrease throttle
a - d:          Increase / Decrease steering
Space:          Stop 
c:              Exit
"""

inc_states = {
    'w': [1.0, 0.0],
    'x': [-1.0, 0.0],
    'd': [0.0, -1.0],
    'a': [0.0, 1.0]
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    sim_diff_drive_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.init_node('keyboard_control')

    throttle_multiplier = rospy.get_param('~throttle_multiplier')
    steering_multiplier = rospy.get_param('~steering_multiplier')

    reset_state = np.array([0.0, 0.0])
    command_state = np.copy(reset_state)

    rospy.loginfo(msg)

    while not rospy.is_shutdown():
        key = getKey()

        if key == 'c':
            exit(0)

        if key not in inc_states.keys() and key != 's':
            continue

        if key in inc_states.keys():
            # Change in command direction causes reset in throttle or steering
            for i in range(len(command_state)):
                if inc_states[key][i] * command_state[i] < -1e-6:
                    command_state[i] = 0.0
                else:
                    command_state[i] = command_state[i] + inc_states[key][i]
        elif key == 's':
            command_state = np.copy(reset_state)

        applied_command = np.multiply(command_state, [throttle_multiplier, steering_multiplier])
        rospy.loginfo("throttle: %.2f, steering=%.2f" % (applied_command[0], applied_command[1]))
        msg = Twist()
        msg.linear.x = applied_command[0]
        msg.angular.z = applied_command[1]
        sim_diff_drive_pub.publish(msg)
