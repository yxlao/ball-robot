#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
rospy.init_node ('Twist', anonymous = True)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
	twist = Twist()
	twist.linear.x = 0.05
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	pub.publish(twist)
	rate.sleep()


		
