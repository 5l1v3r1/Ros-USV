#!/usr/bin/env python
# coding: utf8


from serial import Serial
import rospy
from std_msgs.msg import String
from encoder import Encoder
import threading
import time


class Control:
    def __init__(self, duration, port, baud):
        self.duration = duration
        self.ser = Serial(port,baud)
    
    def start(self):
        self.ser.write("1")
        self.startTime = time.time()
        
        
    def stop(self):
        self.ser.write("2")
        self.stopTime = time.time()
        
        

    def calibration(self):
        ###Sequence ec
        self.ser.write("enterec")
        time.sleep(5)
        self.ser.write("calec")
        time.sleep(2)
        self.ser.write("exitec")
        



