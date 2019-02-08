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
        response = self.ser.readline()
        if response is "OK\n":
            return {True , time.time()}
        else:
            return {False, time.time()}
        
    def stop(self):
        self.ser.write("2")
        self.stopTime = time.time()
        response = self.ser.readline()
        if response is "STOP\n":
            return {True, time.time()}
        else:
            return {False, time.time()}

    def calibration(self):
        ###Sequence ph
        self.ser.write("")
        time.sleep(1)
        self.ser.write("")
        time.sleep(5)
        self.ser.write("Exit")
        ###Sequence orp
        self.ser.write("")
        time.sleep(1)
        self.ser.write("")
        time.sleep(5)
        self.ser.write("Exit")



