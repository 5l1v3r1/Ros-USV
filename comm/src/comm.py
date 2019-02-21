#!/usr/bin/env python
# coding: utf8


import serial
import rospy
from std_msgs.msg import String
from encoder import Encoder
import threading
import time


PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
global ser_mutex
ser_mutex = threading.Lock()
ser = serial.Serial(PORT, BAUD_RATE)
receive_pub = rospy.Publisher("/comm/received_msg", String, queue_size=10)


def get_Xbee(port):
    global ser_mutex
    data = []


    while True:
        ser_mutex.acquire()
        package = port.readline()
        ser_mutex.release()

        if len(package) is not 0:
            content = package.split('/')
            rospy.loginfo("-----XBEE---Received: " + content[1])
            received_message(content[0])


def send_message(msg):
    message = Encoder(msg)
    encoded = message.encode() + "\n"

    ser_mutex.acquire()
    ser.write(encoded)
    ser_mutex.release()
    
    rate.sleep()


def received_message(cont):
    msg = String()
    msg.data = cont
    receive_pub.publish(msg)


check_pub = rospy.Publisher("/monitor/check", String, queue_size=10)

def checker_send(msg):
        if msg is "Request":
                respon = String()
                respon.data = "comm"
                check_pub.publish(respon)


#######################################
########### INIT AND START ############
#######################################

rospy.init_node("comm")

rospy.Subscriber("/comm/send_msg", String, send_message)
rospy.Subscriber("/monitor/check_req", String, checker_send)



readThrd = threading.Thread(target=get_Xbee, args=(ser,))
readThrd.start()


rate = rospy.Rate(10)
while not rospy.is_shutdown():
        rate.sleep()


if rospy.is_shutdown():
    ser.close()

