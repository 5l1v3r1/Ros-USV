#!/usr/bin/env python
# coding: utf8

import rospy
from std_msgs.msg import String
import threading
import time
from gps_common.msg import GPSFix
from gps_common.msg import GPSStatus
from sensor_msgs.msg import NavSatFix
import numpy as np
import math
import json
from encoder import Encoder

GPS_FIX_PUB_TOPIC = "fix"
GPS_EXT_FIX_PUB_TOPIC = "extended_fix"

file_gps = open('gps_pos.txt', 'w', 0)

euc_pub = rospy.Publisher("/nav/dist", String, queue_size=10)
check_pub = rospy.Publisher("/monitor/check", String, queue_size=10)
motor_pub = rospy.Publisher("/motor/cmd", String, queue_size=10)


lat = 0.0
lon = 0.0
R = 6373.0

speed = 0
w = 0
dt = 0.1
T = 0
current_loc_x = 0.0
current_loc_y = 0.0
threshold = 0.5


x_bias = np.zeros((3,1))

kp = 15
kd = 1.5
ki = 0

p = 0
I = 0
D = 0

err = 0
err_p = 0


waypoints = []
ahrs_vals = []
current_speed = 0.0


target_lat = 0.0
target_lon = 0.0

completed = False

with open("route.json") as f:
        data = json.load(f)


 #write the gps into the file
def callback_gps_fix(data):
        
        lat = data.latitude
        lon = data.longitude
        print(data.latitude, data.longitude)
        print(data.position_covariance)

	pos_cov = data.position_covariance

	ts_computer = time.time()
	ts_gps = data.header.stamp.secs
	print(ts_computer, ts_gps)
	data_txt = str(ts_computer) + ' ' + str(ts_gps) + ' ' + str(data.latitude) + ' ' + str(data.longitude)
	for i in range(0, len(pos_cov)):
		data_txt = data_txt + ' ' + str(pos_cov[i]) 

	data_txt = data_txt +'\n'

	file_gps.write(data_txt)



def euclidean_dist():
        first = 0
        last = len(data) - 1
        bestFound = False

        nearest = (0,0)
        
        while first <= last and not bestFound:
                midpoint = (first + last) // 2
                lat1 = math.radians(lat)
                lon1 = math.radians(lon)
                lat2 = math.radians(data[midpoint]["lat"])
                lon2 = math.radians(data[midpoint]["lon"])

                dlon = lon2 - lon1
                dlat = lat2 - lat1
                
                a = (math.sin(dlat/2))**2 + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon/2))**2
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
                dist = R * c * 1000
                min = 100000000000
        
                if dist < min:
                        min = dist 
                        nearest = (dist, midpoint)
                        
                        midpoint = midpoint + 1 
                

        msg = String()
        msg.data = nearest[1] + "/" + nearest[0]
        euc_pub.publish(msg)
        rospy.loginfo("Nearest point is " + nearest[1] + " and distance is " + nearest[0] + " meter")
        


def nav_parameter_listener(msg):
        raw = Encoder(msg).decode()


def checker_send(msg):
        if msg is "Request":
                respon = String()
                respon.data = "navigation"
                check_pub.publish(respon)
                rate.sleep()




def nav_task_handler(msg):
        raw = Encoder(msg).decode()
        splitted = raw.split(",")

        lat = splitted[0]
        lon = splitted[1]

        route_calculator(lat, lon)
        





def task_handler():
        if not tasks:
                while(not tasks):
                        time.sleep(1)
        else:
                current_task = tasks[0]


def route_calculator(lat, lon):



#############################
# This will get lat,lon
# Also this will lock the process

def directRoute(msg):
        raw = Encoder(msg).decode()
        lat = float(raw[0])
        lon = float(raw[1])


def distance_calculator(lat,lon):
        global current_loc_x, current_loc_y

        lat1 = math.radians(lat)
        lon1 = math.radians(lon)
        lat2 = math.radians(current_loc_x)
        lon2 = math.radians(current_loc_y)

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = (math.sin(dlat/2))**2 + math.cos(lat1) * \
            math.cos(lat2) * (math.sin(dlon/2))**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist = R * c * 1000

        return dist


def route_thread(lat,lon):
        global current_speed, current_loc_x, current_loc_y

        dist = distance_calculator(lat,lon)

        remain_time = dist // current_speed

        cmd = String

        dist = distance_calculator(lat,lon)
        heading = float(ahrs_vals[-1][2])

        ### Calculate route

        while dist >= 10:


                
                time.sleep(1)



def route_listener(msg):
        raw = Encoder(msg).decode()
        

                
#################################
### Monitor -> Autonom -> NAV ###
#################################




def ahrs_listener(speed):
        raw = Encoder(speed).decode()

        vals = raw.split(",")

        ahrs_vals.append(vals)

        current_speed = vals[1]




#######################################
########### INIT AND START ############
#######################################




rospy.init_node('Navigation',anonymous=True)

rospy.Subscriber(GPS_FIX_PUB_TOPIC, NavSatFix, callback_gps_fix)
rospy.Subscriber("/monitor/check_req", String, checker_send)
rospy.Subscriber("/automa/route", String, nav_task_handler)
rospy.Subscriber("/ahrs/velocity", String, ahrs_listener)
rospy.Subscriber("/nav/direct", String, directRoute)
rospy.Subscriber("/nav/routeinfo", String, route_listener)





rospy.spin()




#####################
#####  THREADS  #####
#####################


tasks_handling = threading.Thread(target=task_handler)
route_thread = threading.Thread(target=route_thread,args=target_lat,target_lon)
distThread = threading.Thread(target=euclidean_dist)




####################
###### START #######
####################



distThread.start()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
        rate.sleep()
