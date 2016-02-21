#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist
from irobotcreate2.msg import PlaySong
from irobotcreate2.msg import Song
from irobotcreate2.msg import Note
from std_msgs.msg import String

import sys, select, termios, tty

state = "stop"
twist_msg = Twist()
twist_msg.linear.x = 0.0
twist_msg.linear.y = 0.0
twist_msg.linear.z = 0.0
twist_msg.angular.x = 0.0
twist_msg.angular.y = 0.0
twist_msg.angular.z = 0.0

rospy.init_node("create_state_machine", anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
last_command = "s"

def state_change_callback(data):
    global state

    if data.data == "w":
        forward_msg = twist_msg
        forward_msg.linear.x = 0.2
        pub.publish(forward_msg)
        twist_msg.linear.x = 0.0
        state = "drive"
    elif data.data == "a":
        left_msg = twist_msg
        left_msg.angular.z = 0.5
        pub.publish(left_msg)
        twist_msg.angular.z = 0.0
        state = "left"
    elif data.data == "d":
        right_msg = twist_msg
        right_msg.angular.z = -0.5
        pub.publish(right_msg)
        twist_msg.angular.z = 0.0
        state = "right"
    else:
        state == "stop"
        pub.publish(twist_msg)



rospy.Subscriber("/state_cmds", String, state_change_callback)
rospy.spin()