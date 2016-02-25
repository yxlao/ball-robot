#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist, Vector3
from irobotcreate2.msg import PlaySong
from irobotcreate2.msg import Song
from irobotcreate2.msg import Note
from irobotcreate2.msg import RoombaIR
from irobotcreate2.msg import Bumper
from std_msgs.msg import String


import sys, select, termios, tty
import random

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
done_avoiding_obstacle = True
ball_coords = Vector3()
ball_coords.x = 0
ball_coords.y = 0
ball_coords.z = 0


command_start_time = 0.0
explore_duration = 5.0
explore_state = "explore_turn"


rospy.init_node("create_state_machine", anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
last_command = "stop"
rate1 = rospy.Rate(1.0) # 1 sec
rate2 = rospy.Rate(2.0) #0.5 sec


def drive():
    global last_command, command_start_time
    forward_msg = twist_msg
    forward_msg.linear.x = 0.2
    pub.publish(forward_msg)
    last_command = "drive"
    command_start_time = rospy.get_time()
    twist_msg.linear.x = 0.0

def back_up():
    global last_command, command_start_time
    back_msg = twist_msg
    back_msg.linear.x = -0.2
    pub.publish(back_msg)
    last_command = "back"
    command_start_time = rospy.get_time()
    twist_msg.linear.x = 0.0

def turn_left():
    global last_command, command_start_time
    left_msg = twist_msg
    left_msg.angular.z = 0.5
    pub.publish(left_msg)
    last_command = "turn_left"
    command_start_time = rospy.get_time()
    twist_msg.angular.z = 0.0

def turn_right():
    global last_command, command_start_time
    right_msg = twist_msg
    right_msg.angular.z = -0.5
    pub.publish(right_msg)
    last_command = "turn_right"
    command_start_time = rospy.get_time()
    twist_msg.angular.z = 0.0

def stop():
    global last_command, command_start_time
    pub.publish(twist_msg)
    last_command = "stop"
    command_start_time = rospy.get_time()

def explore():
    global last_command, explore_duration
    if last_command == "explore":
        turn_right()
        explore_duration = random.random()*8
        explore_state = "explore_turn"
    elif rospy.get_time() > command_start_time + explore_duration:
        if explore_state == "turn_right":
            drive()
            explore_duration = random.random()*8
            explore_state = "explore_drive"
        else:
            turn_right()
            explore_duration = random.random()*8
            explore_state = "explore_turn"



def ir_bumper_callback(msg):
    global done_avoiding_obstacle, state
    if msg.state and done_avoiding_obstacle:
        state = "avoid"
        done_avoiding_obstacle = False


def ball_in_sight_callback(msg):
    global ball_in_sight, ball_in_sight_changed
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
    ball_coords.x = msg.x
    ball_coords.y = msg.y
    ball_coords.z = msg.z



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
    elif data.data == "x":
        back_up()
        state = "back"
    elif data.data == "q":
        last_command = "find_ball"
        state = "find_ball"
    elif data.data == "e":
        state = "avoid"
        last_command = "avoid"
    elif data.data == "z":
        state = "explore"
        last_command = "explore"
    else:
        stop()
        state = "stop"


def run_state_machine():
    global done_avoiding_obstacle, state
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
    elif state == "back":
        if last_command != "back":
            back_up()
    elif state == "avoid":
        stop()
        rospy.sleep(1)
        back_up()
        rospy.sleep(1.5)
        turn_left()
        rospy.sleep(3)
        drive()
        rospy.sleep(2)
        state = "find_ball"
        done_avoiding_obstacle = True
    elif state == "explore":
        explore()



    elif state == "stop":
        if last_command != "stop":
            stop()


rospy.Subscriber("/ir_bumper", RoombaIR, ir_bumper_callback)
rospy.Subscriber("/state_cmds", String, state_change_callback)
rospy.Subscriber("/is_ball_in_sight", String, ball_in_sight_callback)
rospy.Subscriber("/ball_coords", Vector3, ball_coords_callback)
try:
    while not rospy.is_shutdown():
        run_state_machine()
except rospy.ROSInterruptException:
    pass
