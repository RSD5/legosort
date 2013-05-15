#!/usr/bin/env python

import roslib
roslib.load_manifest('legosort')
import rospy
#from std_msgs.msg import String
from rsd.rsd.msg import RSDMSG

def robotSender(command, boxnumber):
    pub = rospy.Publisher('plc', RDMSG)
    rospy.loginfo("sending to plc "+sendStr)
    msg_to_send = RSDMSG()
    msg_to_send.plccommand = command
    msg_to_send.boxnumber = boxnumber
    pub.publish(msg_to_send)

def robot_callback(msg):
    #message layout; command - parameter
    if msg.robotcommand.lower() == "pickup" :
        robotSender("robot", msg.boxnumber)
    elif msg.robotcommand.lower() == "stop" :
        ser.write('2')
    elif msg.robotcommand.lower() == "reverse":
        ser.write('3')
    elif msg.robotcommand.lower() == "fence":
        ser.write('4')
    #wait for the output buffer to drain
    ser.flush()
    ser.close()


def robot_listener():
    rospy.init_node('robot_controller')
    rospy.Subscriber('robot', String , robot_callback)
    rospy.spin()
    


if __name__ == '__main__':
        try:
            robot_listener()
        except rospy.ROSInterruptException: 
            pass