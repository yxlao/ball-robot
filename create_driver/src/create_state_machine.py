#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist, Vector3
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
ball_in_sight = False
ball_in_sight_changed = False
ball_coords = Vector3()
ball_coords.x = 0
ball_coords.y = 0
ball_coords.z = 0

rospy.init_node("create_state_machine", anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
last_command = "stop"

def drive():
    global last_command
    forward_msg = twist_msg
    forward_msg.linear.x = 0.2
    pub.publish(forward_msg)
    last_command = "drive"
    twist_msg.linear.x = 0.0

def turn_left():
    global last_command
    left_msg = twist_msg
    left_msg.angular.z = 0.5
    pub.publish(left_msg)
    last_command = "turn_left"
    twist_msg.angular.z = 0.0

def turn_right():
    global last_command
    right_msg = twist_msg
    right_msg.angular.z = -0.5
    pub.publish(right_msg)
    last_command = "turn_right"
    twist_msg.angular.z = 0.0

def stop():
    pub.publish(twist_msg)


def ir_bumper_callback(msg):
    a = 1

def ball_in_sight_callback(msg):
    if msg.data == "false":
        if ball_in_sight == True:
            ball_in_sight = False
            ball_in_sight_changed = True
    elif msg.data == "true":
        if ball_in_sight == True:
            a = 1
            #do nothing
        else:
            ball_in_sight = True
            ball_in_sight_changed = True

def ball_coords_callback(msg):
    global ball_coords
    ball_coords.x = msg.data.x
    ball_coords.y = msg.data.y
    ball_coords.z = msg.data.z



def state_change_callback(data):
    global state

    if data.data == "w":
        drive()
        state = "drive"
    elif data.data == "a":
        turn_left()
        state = "left"
    elif data.data == "d":
        turn_right()
        state = "right"
    elif data.data == "q":
        state = "find_ball"
    else:
        stop()
        state = "stop"


def run_state_machine():
    if state == "find_ball":
        if ball_in_sight:
            if last_command != "drive":
                drive()
        else:
            if last_command != "turn_right":
                turn_right()
    elif state == "drive":
        if last_command != "drive":
            drive()

    elif state == "turn_right":
        if last_command != "turn_right":
            turn_right()

    elif state == "turn_left":
        if last_command != "turn_left":
            turn_left()

    elif state == "stop":
        if last_command != "stop":
            stop()



rospy.Subscriber("/state_cmds", String, state_change_callback)
try:
    while not rospy.is_shutdown():
        run_state_machine()
except rospy.ROSInterruptException:
    pass