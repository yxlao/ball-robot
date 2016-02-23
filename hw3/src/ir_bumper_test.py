#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from irobotcreate2.msg import RoombaIR
from irobotcreate2.msg import Bumper
from std_msgs.msg import String


state = "stop"
twist_msg = Twist()
twist_msg.linear.x = 0.0
twist_msg.linear.y = 0.0
twist_msg.linear.z = 0.0
twist_msg.angular.x = 0.0
twist_msg.angular.y = 0.0
twist_msg.angular.z = 0.0

rospy.init_node("ir_bumper_test", anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)


def ir_bumper_callback(data):
    if data.header.frame_id == "base_irbumper_left":
        if data.state == True:
            turn_right_msg = twist_msg
            turn_right_msg.linear.x = 0
            turn_right_msg.angular.z = -0.5
            pub.publish(turn_right_msg)
    elif data.header.frame_id == "base_irbumper_front_left":
        if data.state == True:
            turn_right_msg = twist_msg
            turn_right_msg.linear.x = 0
            turn_right_msg.angular.z = -0.5
            pub.publish(turn_right_msg) 
    elif data.header.frame_id == "base_irbumper_center_left":
        if data.state == True:
            turn_right_msg = twist_msg
            turn_right_msg.linear.x = 0
            turn_right_msg.angular.z = -0.5
            pub.publish(turn_right_msg)
    elif data.header.frame_id == "base_irbumper_right":
        if data.state == True:
            turn_left_msg = twist_msg
            turn_left_msg.linear.x = 0
            turn_left_msg.angular.z = 0.5
            pub.publish(turn_left_msg) 
    elif data.header.frame_id == "base_irbumper_center_right":
        if data.state == True:
            turn_left_msg = twist_msg
            turn_left_msg.linear.x = 0
            turn_left_msg.angular.z = 0.5
            pub.publish(turn_left_msg) 
    elif data.header.frame_id == "base_irbumper_front_right":
        if data.state == True:
            turn_left_msg = twist_msg
            turn_left_msg.linear.x = 0
            turn_left_msg.angular.z = 0.5
            pub.publish(turn_left_msg)

def bumper_callback(data):
    if data.left.state == True or data.right.state == True:
        stop_msg = twist_msg
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0


rospy.Subscriber("/ir_bumper", RoombaIR, ir_bumper_callback)
rospy.Subscriber("/bumper", Bumper, bumper_callback)            
rospy.spin()
