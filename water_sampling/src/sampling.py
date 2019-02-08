#!/usr/bin/env python
# coding: utf8


import serial
import rospy
from std_msgs.msg import String
from encoder import Encoder
from control import Control
import threading
import time
import usb_finder

PORT = "/dev/ttyUSB2" 
#+ usb_finder.USB.find("Arduino")
BAUD_RATE = 9600
global ser_mutex
ser_mutex = threading.Lock()
receive_pub = rospy.Publisher("/water/data", String, queue_size=10)
device = Control(10,PORT, BAUD_RATE)
ser = serial.Serial(PORT, BAUD_RATE)
readThrd = threading.Thread(target=get_sensor, args=(ser,))

old_values = []


def get_sensor(port):
    global ser_mutex

    while True:
        ser_mutex.acquire()
        package = port.readline().decode()
        ser_mutex.release()
 
        if len(package) is not 0:

            content = Encoder(package).decode()
            
            if content:
                #Waiting for accurate data
                rospy.loginfo("-----Water Sampling---Received: " + content)


                old_values.append(package)


                send_message(package)   
            else:
                rospy.loginfo("-----Water Sampling---Finished")
                return
        
            
            
        ##1s delay for equailizing data rates
        time.sleep(1)


check_pub = rospy.Publisher("/monitor/check", String, queue_size=10)


def checker_send(msg):
        if msg is "Request":
                respon = String()
                respon.data = "water_sampler"
                check_pub.publish(respon)


def command(msg):

#############
#CMD will be
# {start}

    if msg is "start":
        
        device.start()
        time.sleep(10)
        device.calibration()
        rospy.loginfo("-----Water Sampling---Calibrated")
        time.sleep(3)
        readThrd.start()

        rospy.loginfo("Sampling started" + time.time())
        
    elif msg is "stop":
        if readThrd.isAlive():
                #If thread is not alive no mean to send stop sign to device
                device.stop()

                #Waiting for securing the serial port connection
                time.sleep(1)



def stopper(msg):
        raw = Encoder(msg).decode()

        components = raw.split(",")

        for i in range(len(components)):
                if "water_sampler" is components[i]:
                        device.stop()



                

def send_message(cont):
    msg = String()
    msg.data = cont
    receive_pub.publish(msg)


#######################################
########### INIT AND START ############
#######################################

rospy.init_node("waterSampler", anonymous=True)

rospy.Subscriber("/water/command", String, command)
rospy.Subscriber("/monitor/check_req", String, checker_send)
rospy.Subscriber("/stopper", String, stopper)


rospy.spin()


rate = rospy.Rate(10)
while not rospy.is_shutdown():
        rate.sleep()


if rospy.is_shutdown():
    ser.close()

