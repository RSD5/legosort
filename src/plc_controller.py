#!/usr/bin/env python

import roslib
roslib.load_manifest('legosort')
import rospy
#from std_msgs.msg import String
from rsd.rsd.msg import RSDMSG
import serial

COMPORT = '/dev/ttyS1'
BAUDRATE = 19200

def plc_callback(msg):
    ser = serial.Serial(COMPORT, BAUDRATE, timeout=1)
    if msg.plccommand.lower() == "start" :
        ser.write('1')
    elif msg.plccommand.lower() == "stop" :
        ser.write('2')
    elif msg.plccommand.lower() == "reverse":
        ser.write('3')
    elif msg.plccommand.lower() == "fence":
        ser.write('4')
    elif msg.plccommand.lower() == "robot":
        if msg.boxnumber == 1:
            ser.write('5')
        elif msg.boxnumber == 2:
            ser.write('6')
        elif msg.boxnumber == 3:
            ser.write('7')
        elif msg.boxnumber == 4:
            ser.write('8')
        elif msg.boxnumber == 5:
            ser.write('9')
        elif msg.boxnumber == 6:
            ser.write('0')
        elif msg.boxnumber == 7:
            ser.write('a')
        elif msg.boxnumber == 8:
            ser.write('b')
        elif msg.boxnumber == 9:
            ser.write('c')
        elif msg.boxnumber == 10:
            ser.write('d')
        elif msg.boxnumber == 11:
            ser.write('e')
        elif msg.boxnumber == 12:
            ser.write('f')
        elif msg.boxnumber == 13:
            ser.write('g')
        elif msg.boxnumber == 14:
            ser.write('h')
        elif msg.boxnumber == 15:
            ser.write('i')
        elif msg.boxnumber == 16:
            ser.write('j')


    #wait for the output buffer to drain
    ser.flush()
    ser.close()


def conveyor_listener():
    rospy.init_node('plc_controller')
    rospy.Subscriber('plc', RSDMSG , plc_callback)
    rospy.spin()
    


if __name__ == '__main__':
        try:
            conveyor_listener()
        except rospy.ROSInterruptException: 
            pass