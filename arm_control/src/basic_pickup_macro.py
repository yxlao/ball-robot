#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

import os
import sys
import termios
import fcntl


def getch():
    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

    try:
        while 1:
            try:
                c = sys.stdin.read(1)
                break
            except IOError:
                pass
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
    return c
cmd_queue = ['w', 'w', 'w', 'c', 's', 's', 's', 'o']


def talker():
    pub = rospy.Publisher('arm_cmds', String, queue_size=10)
    arm_macro_command_pub = rospy.Publisher('should_pick_up_ball', String, queue_size=3)
    should_drop_ball_pub = rospy.Publisher('/should_drop_ball', String, queue_size=4)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    while not rospy.is_shutdown():
        hello_str = getch()
        print hello_str
        if hello_str == '1':
            arm_macro_command_pub.publish("true")
        elif hello_str == '2':
            arm_macro_command_pub.publish("false")
        elif hello_str == '3':
            should_drop_ball_pub.publish("true")
        elif hello_str == "4":
            arm_macro_command_pub.publish("reset")
        else:
            # hello_str = "forward"
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
