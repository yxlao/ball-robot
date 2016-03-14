#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist, Vector3
from irobotcreate2.msg import PlaySong
from irobotcreate2.msg import Song
from irobotcreate2.msg import Note
from irobotcreate2.msg import RoombaIR
from irobotcreate2.msg import Bumper
from std_msgs.msg import String, UInt8


import sys
import select
import termios
import tty
import random

FAST_TURN_SPEED = 0.5
SLOW_TURN_SPEED = 0.1
FAST_DRIVE_SPEED = 0.15
SLOW_DRIVE_SPEED = 0.08
state = "stop"
twist_msg = Twist()
twist_msg.linear.x = 0.0
twist_msg.linear.y = 0.0
twist_msg.linear.z = 0.0
twist_msg.angular.x = 0.0
twist_msg.angular.y = 0.0
twist_msg.angular.z = 0.0
ball_in_sight = False
lower_cam_ball_in_sight = False
ball_in_sight_changed = False
is_ball_in_claw = False
bucket_in_sight = False
lower_cam_ball_in_sight_changed = False
done_avoiding_obstacle = True
back_obstructed = False
ball_coords = Vector3()
ball_coords.x = 0
ball_coords.y = 0
ball_coords.z = 0
lower_cam_ball_coords = Vector3()
lower_cam_ball_coords.x = 0
lower_cam_ball_coords.y = 0
lower_cam_ball_coords.z = 0

bucket_coords = Vector3()
bucket_coords.x = 0
bucket_coords.y = 0
bucket_coords.z = 0

command_start_time = 0.0
explore_duration = 5.0
explore_state = "explore_turn"
avoid_state = "stop"




rospy.init_node("create_state_machine", anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
should_drop_ball_pub = rospy.Publisher('/should_drop_ball', String, queue_size=3)
pick_up_pub = rospy.Publisher('/should_pick_up_ball', String, queue_size=3)
state_pub = rospy.Publisher('/state', String, queue_size=3)

last_command = "stop"
rate1 = rospy.Rate(1.0)  # 1 sec
rate2 = rospy.Rate(3.0)  # 0.5 sec
time_since_ball_in_center = rospy.get_time()
time_since_bucket_in_center = rospy.get_time()

def drive(speed=FAST_DRIVE_SPEED):
    global last_command, command_start_time
    forward_msg = twist_msg
    forward_msg.linear.x = speed
    pub.publish(forward_msg)
    last_command = "drive"
    command_start_time = rospy.get_time()
    twist_msg.linear.x = 0.0


def back_up(speed=FAST_DRIVE_SPEED):
    global avoid_state, back_obstructed, last_command, state, command_start_time
    back_msg = twist_msg
    back_msg.linear.x = -1 * speed
    if back_obstructed:
        avoid_state = "turn_left"
        state = "avoid"
    pub.publish(back_msg)
    last_command = "back"
    command_start_time = rospy.get_time()
    twist_msg.linear.x = 0.0


def turn_left(speed=FAST_TURN_SPEED):
    global last_command, command_start_time
    left_msg = twist_msg
    left_msg.angular.z = speed
    pub.publish(left_msg)
    last_command = "turn_left"
    command_start_time = rospy.get_time()
    twist_msg.angular.z = 0.0


def turn_right(speed=FAST_TURN_SPEED):
    global last_command, command_start_time
    right_msg = twist_msg
    right_msg.angular.z = -1 * speed
    pub.publish(right_msg)
    last_command = "turn_right"
    command_start_time = rospy.get_time()
    twist_msg.angular.z = 0.0


def stop():
    global last_command, command_start_time
    pub.publish(twist_msg)
    last_command = "stop"
    command_start_time = rospy.get_time()

def fine_right_turn():
    turn_right(speed=SLOW_TURN_SPEED)
    rate2.sleep()
    stop()
    rate2.sleep()

def fine_left_turn():
    turn_left(speed=SLOW_TURN_SPEED)
    rate2.sleep()
    stop()
    rate2.sleep()


def explore():
    global last_command, explore_duration, explore_state, command_start_time
    if last_command == "explore":
        turn_right()
        explore_duration = random.random() * 8
        explore_state = "explore_turn"
    elif rospy.get_time() > command_start_time + explore_duration:
        if explore_state == "explore_turn":
            drive()
            explore_duration = random.random() * 8
            explore_state = "explore_drive"
        else:
            turn_right()
            explore_duration = random.random() * 8
            explore_state = "explore_turn"


def fine_position():
    global last_command, command_start_time, state, time_since_ball_in_center
    if abs(lower_cam_ball_coords.x) < 0.3:
        if lower_cam_ball_coords.y < 13:
            if lower_cam_ball_coords.y < 5:
                back_up(speed=SLOW_DRIVE_SPEED)
                time_since_ball_in_center = rospy.get_time()
            else:
                stop()
                if rospy.get_time() - time_since_ball_in_center > 3:
                    state = "pick_up"
                    pick_up_pub.publish('true')
                    rospy.sleep(15)
                    state = "find_bucket"
        else:
                    drive(speed=SLOW_DRIVE_SPEED)
    elif lower_cam_ball_coords. x >= 0.5:
            time_since_ball_in_center = rospy.get_time()
            fine_right_turn()
    else:
            time_since_ball_in_center = rospy.get_time()
            fine_left_turn()
    command_start_time = rospy.get_time()

def fine_bucket_position():
    global last_command, command_start_time, state, time_since_bucket_in_center, bucket_coords
    if abs(bucket_coords.x) < 0.5:
        if bucket_coords.y < 15:
                    stop()
                    if rospy.get_time() - time_since_bucket_in_center > 1:
                        state = "stop"
                        #pick_up_pub.publish('true')
        else:
                    drive(speed=SLOW_DRIVE_SPEED)
    elif bucket_coords.x >= 0.5:
            time_since_bucket_in_center = rospy.get_time()
            fine_right_turn()
    else:
            time_since_ball_in_center = rospy.get_time()
            fine_left_turn()
    command_start_time = rospy.get_time()

def avoid():
    global avoid_state, done_avoiding_obstacle, state, last_command
    current_time = rospy.get_time()
    if last_command == "avoid":
        avoid_state = "stop"
        stop()
    elif avoid_state == "stop":
        if current_time - command_start_time > 1:
            back_up()
            avoid_state = "back_up"
    elif avoid_state == "back_up":
         if current_time - command_start_time > 1.5:
            turn_left()
            avoid_state = "turn_left"
    elif avoid_state == "turn_left":
        if current_time - command_start_time > 3:
            drive()
            avoid_state = "drive"
    elif avoid_state == "drive":
        if current_time - command_start_time > 2:
            state = "explore"
            last_command = "explore"
            avoid_state = "stop"
            done_avoiding_obstacle = True



def avoid():
    global avoid_state, done_avoiding_obstacle, state, last_command
    current_time = rospy.get_time()
    if last_command == "avoid":
        avoid_state = "stop"
        stop()
    elif avoid_state == "stop":
        if current_time - command_start_time > 1:
            back_up()
            avoid_state = "back_up"
    elif avoid_state == "back_up":
         if current_time - command_start_time > 1.5:
            turn_left()
            avoid_state = "turn_left"
    elif avoid_state == "turn_left":
        if current_time - command_start_time > 3:
            drive()
            avoid_state = "drive"
    elif avoid_state == "drive":
        if current_time - command_start_time > 2:
            state = "explore"
            last_command = "explore"
            avoid_state = "stop"
            done_avoiding_obstacle = True


def ir_bumper_callback(msg):
    global done_avoiding_obstacle, state, last_command
    if msg.state and done_avoiding_obstacle:
        state = "avoid"
        last_command = "avoid"
        done_avoiding_obstacle = False


def ball_in_sight_callback(msg):
    global ball_in_sight, ball_in_sight_changed, done_avoiding_obstacle, state, lower_cam_ball_in_sight
    if msg.data == "false":
        if ball_in_sight == True:
            ball_in_sight = False
            ball_in_sight_changed = True
    elif msg.data == "true":
        if (not lower_cam_ball_in_sight) and done_avoiding_obstacle and state != "pick_up" and state !="find_bucket" :
            state = "find_ball"
        if ball_in_sight == True:
            a = 1
            # do nothing
        else:
            ball_in_sight = True
            ball_in_sight_changed = True

def lower_cam_ball_in_sight_callback(msg):
    global lower_cam_ball_in_sight, lower_cam_ball_in_sight_changed,state
    if msg.data == "false":
        if lower_cam_ball_in_sight == True:
            lower_cam_ball_in_sight = False
            lower_cam_ball_in_sight_changed = True
    elif msg.data == "true":
        if lower_cam_ball_in_sight == True:
            a = 1
            # do nothing
        else:
            if state == "avoid":
                return
            if state == "pick_up":
                return
            if state == "find_bucket":
                return

            state = "fine_position"
            lower_cam_ball_in_sight = True
            lower_cam_ball_in_sight_changed = True

def front_ultrasonic_callback(msg):
    global state, should_drop_ball_pub
    if msg.data != 0:
        if msg.data < 20 and state == "find_bucket" and bucket_in_sight and abs(bucket_coords.x) < 0.5:
            should_drop_ball_pub.publish("true")
            state = "stop"

def back_ultrasonic_callback(msg):
    global state, back_obstructed
    if msg.data < 20 and msg.data > 0:
        back_obstructed = True
    else:
        back_obstructed = False

def bucket_in_sight_callback(msg):
    global bucket_in_sight, state
    if msg.data == "true":
        bucket_in_sight = True
    else:
        bucket_in_sight = False

def ball_coords_callback(msg):
    global ball_coords
    ball_coords.x = msg.x
    ball_coords.y = msg.y
    ball_coords.z = msg.z

def lower_cam_ball_coords_callback(msg):
    global lower_cam_ball_coords
    lower_cam_ball_coords.x = msg.x
    lower_cam_ball_coords.y = msg.y
    lower_cam_ball_coords.z = msg.z

def bucket_coords_callback(msg):
    global bucket_coords
    bucket_coords.x = msg.x
    bucket_coords.y = msg.y
    bucket_coords.z = msg.z



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
    elif data.data == "c":
        state = "fine_position"
        last_command = "fine_position"
    elif data.data == "p":
        state= "pick_up"
    elif data.data == "b":
        state="find_bucket"
    else:
        stop()
        state = "stop"


def run_state_machine():
    global done_avoiding_obstacle, state, state_pub, last_command, bucket_in_sight, rate2
    state_pub.publish(state)
    if state == "find_ball":
        if ball_in_sight:
            #if last_command != "drive":
            drive()
            rate2.sleep()
        else:
            #if last_command != "turn_right":
            turn_right()
            rate2.sleep()
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
        avoid()
    elif state == "explore":
        explore()
    elif state == "fine_position":
        if not lower_cam_ball_in_sight:
            state = "explore"
            last_command = "explore"
        else:
            fine_position()
    elif state == "find_bucket":
        if bucket_in_sight:
            fine_bucket_position()
        else:
            if last_command != "turn_right":
                turn_right()



    elif state == "stop":
        if last_command != "stop":
            stop()


#rospy.Subscriber("/ir_bumper", RoombaIR, ir_bumper_callback)
rospy.Subscriber("/state_cmds", String, state_change_callback)
rospy.Subscriber("/is_ball_in_sight", String, ball_in_sight_callback)
rospy.Subscriber("/lower_cam_is_ball_in_sight", String, lower_cam_ball_in_sight_callback)
rospy.Subscriber("/ball_coords", Vector3, ball_coords_callback)
rospy.Subscriber("/lower_cam_ball_coords", Vector3, lower_cam_ball_coords_callback)
rospy.Subscriber("/front_ultrasonic", UInt8, front_ultrasonic_callback)
rospy.Subscriber("/back_ultrasonic", UInt8, back_ultrasonic_callback)
rospy.Subscriber("/is_bucket_in_sight", String, bucket_in_sight_callback)
rospy.Subscriber("/bucket_coords", Vector3, bucket_coords_callback)
try:
    while not rospy.is_shutdown():
        run_state_machine()
except rospy.ROSInterruptException:
    pass
