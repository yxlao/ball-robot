#!/usr/bin/python

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
      except IOError: pass
  finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
  return c

def talker():
    pub = rospy.Publisher('state_cmds', String, queue_size=10)
    rospy.init_node('state_talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        hello_str = getch()
        print hello_str
        # hello_str = "forward"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass