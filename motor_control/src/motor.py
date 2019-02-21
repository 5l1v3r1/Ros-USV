#!/usr/bin/env python
# coding: utf8

import rospy
from std_msgs.msg import String
import threading
import time
from serial import Serial
from usb_finder import USB
from encoder import Encoder


check_pub = rospy.Publisher('/monitor/check', String, queue_size=10)

mon_pub = rospy.Publisher('/comm/send_msg', String, queue_size=10)
mot_pub = rospy.Publisher('/motor/state', String, queue_size=10)


motor_ser = Serial('/dev/serial2',  9600)

def check_usb():
         
        dev = USB.find('Arduino')
        if dev is None:
                rospy.loginfo('Fault with communicating ' + ' MOTOR CONTROL')
        else:
                msg = String()
                msg.data = '--Started to communicate with Motor Control Unit--'
                mon_pub.publish(msg)

                rate.sleep()

                return True
                

def motor_state_listener():
        motor = motor_ser.readline()
        msg =  String()
        msg.data = motor
        mot_pub.publish(msg)

        rate.sleep()


def motor_command(msg):
        raw = msg
        cmds = raw.split(',')
        ######
        #Cmd will be {transmission,angle}
        #revert is 0
        #forward is 1
        cmd = cmds[0] +','+ cmds[1] +'\n' 
        motor_ser.write(cmd)


def checker_send(msg):
        if msg is 'Request':
                respon = String()
                respon.data = 'motor_control'
                check_pub.publish(respon)
                
                rate.sleep()


#######################################
########### INIT AND START ############
#######################################
motorThrd = threading.Thread(target=motor_state_listener)
rospy.init_node('motor_controller')

available = check_usb()

def stopper(msg):
        raw = msg
        parts = raw.split(',')
        for i in range(len(parts)):
                if 'motor_control' is parts[i]:
                        motor_ser.write('stop\n')

                        state = String()
                        state.data = 'Motor-Stopped'
                        mot_pub.publish(state)

                        motorThrd._stop()
                        motor_ser.close()
                        


if motor_ser.is_open:
        motorThrd.start()


rospy.Subscriber('/monitor/check_req', String, checker_send)
rospy.Subscriber('/motor/cmd', String, motor_command)
rospy.Subscriber('/stopper', String, stopper)

rospy.spin()


rate = rospy.Rate(10)
while not rospy.is_shutdown():
        rate.sleep()
