#!/usr/bin/env python
import rospy
import os
import sys    
import termios
import fcntl
from std_msgs.msg import Float32
from std_msgs.msg import String

angle1 = 0.0
angle2 = 0.0
angle3 = 0.0
angle1ready = False
angle2ready = False
angle3ready = False
claw_is_open = True
state = "lowering"
moving_joint = 0

#lowering
angle1above = -74
angle1below = -83
angle2above = 9
angle2below = -1
angle3above = -10
angle3below = -20

#raising
raising_angle1above = -10
raising_angle1below = -20
raising_angle2above = 40
raising_angle2below = 20
raising_angle3above = -10
raising_angle3below = -20

def angle1callback(msg):
    global angle1, angle1ready
    angle1 = msg.data
    angle1ready = True

def angle2callback(msg):
    global angle2, angle2ready
    angle2 = msg.data
    angle2ready = True

def angle3callback(msg):
    global angle3, angle3ready
    angle3 = msg.data
    angle3ready = True

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
      except IOError: pass
  finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
  return c
def run_state_machine():
    a = 1

def get_lowering_cmd():
    global claw_is_open, moving_joint
    if float(angle1) > angle1above:
        moving_joint = 1
        return 's'
    if float(angle1) < angle1below:
        moving_joint = 1
        return 'w'
    if moving_joint == 1:
        moving_joint = 0
        return 'x'
    # if float(angle2) > angle2above:
    #     moving_joint = 2
    #     return 'd'
    # if float(angle2) < angle2below:
    #     moving_joint = 2
    #     return 'e'
    # if moving_joint == 2:
    #     moving_joint = 0
    #     return 'x'
    if float(angle3) > angle3above:
        moving_joint = 3
        return 'f'
    if float(angle3) < angle3below:
        moving_joint = 3
        return 'r'
    if moving_joint == 3:
        moving_joint = 0
        return 'x'
    if claw_is_open:
        claw_is_open = False
        return 'c'
    return "done"
def get_raising_cmd():
    global claw_is_open, moving_joint
    if float(angle1) > raising_angle1above:
        moving_joint = 1
        return 's'
    if float(angle1) < raising_angle1below:
        moving_joint = 1
        return 'w'
    if moving_joint == 1:
        moving_joint = 0
        return 'x'
    # if float(angle2) > raising_angle2above:
    #     moving_joint = 2
    #     return 'g'
    # if float(angle2) < raising_angle2below:
    #     moving_joint = 2
    #     return 't'
    # if moving_joint == 2:
    #     moving_joint = 0
    #     return 'x'
    if float(angle3) > raising_angle3above:
        moving_joint = 3
        return 'f'
    if float(angle3) < raising_angle3below:
        moving_joint = 3
        return 'r'
    if moving_joint == 3:
        moving_joint = 0
        return 'x'
    if not claw_is_open:
        claw_is_open = True
        return 'o'
    return "done"

def run_node():
    global claw_is_open, state
    pub1 = rospy.Publisher('/arm_cmds', String, queue_size=10)
    rospy.Subscriber("/joint1/angle", Float32, angle1callback)
    rospy.Subscriber("/joint2/angle", Float32, angle2callback)
    rospy.Subscriber("/joint3/angle", Float32, angle3callback)
    rospy.init_node('arm_feedback_node', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if not (angle1ready and angle2ready and angle3ready):
            continue
        if state == "lowering":
            next_cmd = str(get_lowering_cmd())
            if next_cmd == "done":
                state = "raising"
                continue
            pub1.publish(next_cmd)
            rate.sleep()
            continue
        if state == 'raising':
            next_cmd = str(get_raising_cmd())
            if next_cmd == "done":
                state = "idle"
                continue
            pub1.publish(next_cmd)
            rate.sleep()
            continue
        else:
            rate.sleep()





try:
    run_node()
except rospy.ROSInterruptException:
    pass
