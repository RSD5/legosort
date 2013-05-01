#!/usr/bin/env python

import roslib
roslib.load_manifest('legosort')
import rospy
from std_msgs.msg import String
import serial

COMPORT = '/dev/ttyS1'
BAUDRATE = 19200


#def testSender():
#    pub = rospy.Publisher('plc',String)
#    rospy.init_node('plctalker')
#    while not rospy.is_shutdown():
#        str = "start"
#        rospy.loginfo(str)
#        pub.publish(String(str))
#        rospy.sleep(1.0)


def plc_callback(msg):
    ser = serial.Serial(COMPORT, BAUDRATE, timeout=1)
    ser.open()
    if msg.data.lower() == "start" :
        ser.write('1')
    elif msg.data.lower() == "stop" :
        ser.write('2')
    elif msg.data.lower() == "reverse":
        ser.write('3')
    elif msg.data.lower() == "fence":
        ser.write('4')
    #wait for the output buffer to drain
    ser.flush()
    ser.close()


def conveyor_listener():
    rospy.init_node('plc_listener')
    rospy.Subscriber('plc', String , plc_callback)
    rospy.spin()
    


if __name__ == '__main__':
        try:
            testSender()
            conveyor()
        except rospy.ROSInterruptException: 
            pass