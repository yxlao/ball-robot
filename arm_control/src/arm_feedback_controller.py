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
should_pick_up_ball = False
should_stop_lowering = False
done_picking_up_ball = True
claw_is_open = True
should_reset = False
state = "idle"
moving_joint = 0

# lowering
angle1above = -66
angle1below = -78
angle2above = 9
angle2below = -1
angle3above = -40
angle3below = -50

# raising
raising_angle1above = 40
raising_angle1below = 27
raising_angle2above = 40
raising_angle2below = 20
raising_angle3above = -40
raising_angle3below = -50

drop_angle1above = 0
drop_angle1below = -25
drop_angle2above = 40
drop_angle2below = 20
drop_angle3above = -40
drop_angle3below = -50

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


def should_pick_up_ball_callback(msg):
    global should_pick_up_ball, should_stop_lowering
    if msg.data == "true":
        should_pick_up_ball = True
    elif msg.data == "false":
        should_stop_lowering = True
    elif msg.data == "reset":
        should_reset = True

def should_drop_ball_callback(msg):
    global pub1, state, claw_is_open
    if msg.data == "true":
        state = "dropping"
    elif msg.data == "false":
        pub1.publish('c')
    elif msg.data == "truefast":
        claw_is_open = True
        pub1.publish('o')


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


def get_lowering_cmd():
    global claw_is_open, moving_joint, should_stop_lowering
    if should_stop_lowering:
        should_stop_lowering = False
        return "done"
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

def get_dropping_cmd():
    global claw_is_open, moving_joint, should_stop_lowering, should_reset
    if should_reset:
        should_reset = False
        state = "raising"
        return "o"
    if float(angle1) > drop_angle1above:
        moving_joint = 1
        return 's'
    if float(angle1) < drop_angle1below:
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
    if float(angle3) > drop_angle3above:
        moving_joint = 3
        return 'f'
    if float(angle3) < drop_angle3below:
        moving_joint = 3
        return 'r'
    if moving_joint == 3:
        moving_joint = 0
        return 'x'
    if not claw_is_open:
        claw_is_open = True
        return 'o'
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
    return "done"


def run_node():
    global claw_is_open, angle1ready, angle2ready, angle3ready, should_pick_up_ball, done_picking_up_ball, state, pickup_status_pub

    while not rospy.is_shutdown():
        if not (angle1ready and angle2ready and angle3ready):
            continue
        if should_pick_up_ball and done_picking_up_ball:
            pickup_status_pub.publish("starting")
            state = "lowering"
            done_picking_up_ball = False
        run_arm_macro()


def run_arm_macro():
    global claw_is_open, state, done_picking_up_ball, pub1, should_pick_up_ball, pickup_status_pub

    rate = rospy.Rate(2)
    if state == "lowering":
        next_cmd = str(get_lowering_cmd())
        if next_cmd == "done":
            pickup_status_pub.publish("raising")
            state = "ball_raising"
            return
        pub1.publish(next_cmd)
        rate.sleep()
        return
    elif state == "dropping":
        next_cmd = str(get_dropping_cmd())
        if next_cmd == "done":
            pickup_status_pub.publish("done")
            state = "raising"
            return
        pub1.publish(next_cmd)
        rate.sleep()
    elif state == 'ball_raising':
        next_cmd = str(get_raising_cmd())
        if next_cmd == "done":
            state = "idle"
            done_picking_up_ball = True
            should_pick_up_ball = False
            pickup_status_pub.publish("drop ready")
            return
        pub1.publish(next_cmd)
        rate.sleep()
    elif state == 'raising':
        next_cmd = str(get_raising_cmd())
        if next_cmd == "done":
            state = "idle"
            done_picking_up_ball = True
            should_pick_up_ball = False
            pickup_status_pub.publish("raised")
            return
        pub1.publish(next_cmd)
        rate.sleep()
    else:
        rate.sleep()


try:
    pub1 = rospy.Publisher('/arm_cmds', String, queue_size=10)
    pickup_status_pub = rospy.Publisher("/pickup_status", String, queue_size=3)
    rospy.Subscriber("/joint1/angle", Float32, angle1callback)
    rospy.Subscriber("/joint2/angle", Float32, angle2callback)
    rospy.Subscriber("/joint3/angle", Float32, angle3callback)
    rospy.Subscriber("/should_pick_up_ball", String, should_pick_up_ball_callback)
    rospy.Subscriber("/should_drop_ball", String, should_drop_ball_callback)
    rospy.init_node('arm_feedback_node', anonymous=True)
    run_node()
except rospy.ROSInterruptException:
    pass
