#!/usr/bin/env python
# license removed for brevity
import serial
import time
import struct
import rospy
from std_msgs.msg import Float32

port = serial.Serial("/dev/ttyACM1", timeout=3.0)

print "running"

angle_90 = 0.4139
angle_0 = 0.115

def voltage_to_angle(voltage):
    return (voltage - angle_0) / (angle_90 - angle_0) * 90. + 0.

pub = rospy.Publisher('arm_angle', Float32, queue_size=10)
rospy.init_node('angle_sender', anonymous=True)
while True:
    
    port.flushInput()
    rcv = port.read(4)
    msg = Float32()
    try:
        v = struct.unpack('f', rcv)[0]
        msg.data = voltage_to_angle(v)
        pub.publish(msg)
        print "voltage: %s, angle: %s" % (v,  voltage_to_angle(v))
    except:
        print "no signal"
    time.sleep(0.2)
    
