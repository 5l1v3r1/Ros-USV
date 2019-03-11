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

GPS_FIX_PUB_TOPIC = 'fix'
GPS_EXT_FIX_PUB_TOPIC = 'extended_fix'

file_gps = open('gps_pos.txt', 'w', 0)

euc_pub = rospy.Publisher('/nav/dist', String, queue_size=10)
check_pub = rospy.Publisher('/monitor/check', String, queue_size=10)
motor_pub = rospy.Publisher('/motor/cmd', String, queue_size=10)



R = 6373.0

speed = 0
w = 0
dt = 0.1
T = 0
current_lon = 0.0
current_lat = 0.0
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

data = []

loc_vals = []


## No route file 
# with open('route.json') as f:
#        data = json.load(f)


# write the gps into the file


def callback_gps_fix(data):

        global current_lat, current_lon
        
        lat = data.latitude
        current_lat = data.latitude
        lon = data.longitude
        current_lon = data.longitude
        print(data.latitude)
        print(data.longitude)
        print(data.position_covariance)

        loc_vals.append([lat, lon])

        pos_cov = data.p-osition_covariance

        ts_computer = time.time()
        ts_gps = data.header.stamp.secs
        print(ts_computer, ts_gps)
        data_txt = str(ts_computer) + ' ' + str(ts_gps) + ' ' + str(data.latitude) + ' ' + str(data.longitude)
        for i in range(0, len(pos_cov)):
	        data_txt = data_txt + ' ' + str(pos_cov[i]) 

        data_txt = data_txt +'\n'


        file_gps.write(data_txt)



def euclidean_dist():
        global data
        first = 0
        last = len(data) - 1
        bestFound = False

        nearest = (0,0)

        lat = current_lon
        lon = current_lat

        while first <= last and not bestFound:
                midpoint = (first + last) // 2
                lat1 = math.radians(lat)
                lon1 = math.radians(lon)
                lat2 = math.radians(data[midpoint]['lat'])
                lon2 = math.radians(data[midpoint]['lon'])

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
        msg.data = nearest[1] + '/' + nearest[0]
        euc_pub.publish(msg)
        rospy.loginfo('Nearest point is ' + nearest[1] + ' and distance is ' + nearest[0] + ' meter')
        




def nav_parameter_listener(msg):
        raw = msg.data





def checker_send(msg):
        if msg.data is 'Request':
                respon = String()
                respon.data = 'navigation'
                check_pub.publish(respon)
                rate.sleep()






def nav_task_handler(msg):
        global target_lat, target_lon

        rospy.loginfo("Nav task handler started queue tasks")

        raw = msg.data
        splitted = raw.split(',')

        target_lat = splitted[0]
        target_lon = splitted[1]

        route_thread.start()

        waypoints.append(target_lat,target_lon)
        


#def task_handler():
#        if not tasks:
 #               while(not tasks):
  #                      time.sleep(1)
   #     else:
    #            current_task = tasks[0]





#############################
# This will get lat,lon
# Also this will lock the process



def directRoute(msg):
        raw = msg.data
        lat = float(raw[0])
        lon = float(raw[1])

        



def distance_calculator(lat,lon):
        global current_lon, current_lat 

        lat1 = math.radians(float(lat))
        lon1 = math.radians(float(lon))
        lat2 = math.radians(float(current_lon))
        lon2 = math.radians(float(current_lat))

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = (math.sin(dlat/2))**2 + math.cos(lat1) * \
            math.cos(lat2) * (math.sin(dlon/2))**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist = R * c * 1000

        return dist





def getLocal(lat, lon):
        WORLD_POLAR_M = 6356752.3142
        WORLD_EQUATORIAL_M = 6378137.0

        eccentricity = math.acos(WORLD_POLAR_M/WORLD_EQUATORIAL_M)
        n_prime = 1/(math.sqrt(1 - math.pow(math.sin(math.radians(float(lat))),
                                            2.0)*math.pow(math.sin(eccentricity), 2.0)))
        m = WORLD_EQUATORIAL_M * \
            math.pow(math.cos(eccentricity), 2.0) * math.pow(n_prime, 3.0)
        n = WORLD_EQUATORIAL_M * n_prime

        if not waypoints:
                home_lon = 45.0
                home_lat = 44.0
        else:
                home_lon = waypoints[3][0]
                home_lat = waypoints[3][1]
 

        difflon = float(lon) - home_lon
        difflat = float(lat) - home_lat


        surfdistLon = math.pi /180.0 * math.cos(math.radians(float(lat))) * n
        surfdistLat = math.pi/180.00 * m
        
        x = difflon * surfdistLon
        y = difflat * surfdistLat

        return x,y 








def route_thr(lat,lon):
        global current_speed, current_lon, current_lat, T, err_p, I 

        rospy.loginfo("Data is reached to route thread")



        dist = distance_calculator(float(lat),float(lon))

        remain_time = float(dist) // float(current_speed)
        #rospy.loginfo("Remained time " + str(remain_time))
        cmd = String()

        dist = distance_calculator(float(lat),float(lon))
        #heading = float(ahrs_vals[-1][2])
        heading = ahrs_vals[-1]

        ### Calculate route

        #home_x = loc_vals[5][0]
        #home_y = loc_vals[5][1]

        

        coord_x, coord_y = getLocal(current_lat, current_lon)

        target_x, target_y = getLocal(float(lat), float(lon))

        #target_x = (lat - home_x)*10
        #target_y = (lon - home_y)*10

        cont = True
        


        while cont:

                if dist < 5:
                        cont = False

                T = T + dt
                x_bias[0] = current_lon
                x_bias[1] = current_lat
                #x_bias[2] = x_bias[2] + w * dt

                x_bias[2] = heading
        
                head_sum = - x_bias[2] + math.atan2(coord_y - x_bias[1], coord_x - x_bias[0])

                desired_heading = math.atan2(math.sin(head_sum), math.cos(head_sum))

                #if np.linalg.norm((x_bias[1:2] - [coord_x, coord_y]), 2) < threshold :
                
                #net = heading - desired_heading

                #if net < 0:
                #        desired_heading = desired_heading - heading
                #else : 
                #        desired_heading = heading - desired_heading
                

                # PID

                err = desired_heading
                P = kp * err
                D = kd *(err - err_p) / dt
                I = I + (ki * err) *dt
                err_p = err

                steer_cmd = P + I + D


                if steer_cmd > 40 or err > math.radians(15):
                        steer_cmd = 10
                else:
                        if steer_cmd < -40 or err < math.radians(-15):
                                steer_cmd = -40


                w = steer_cmd * (math.radians(15) / 40)


                command = String()
                command.data = str(3) + ',' + str(w) 
                motor_pub.publish(command)
                
                rospy.loginfo( str(3) + ',' + str(w) )

                time.sleep(1)


                
        stop = String()
        stop.data = + str(0) + ',' + str(0) 
        rospy.loginfo("Stop data passed to route thread")
        motor_pub.publish(stop)
        
        return

        

route_thread = threading.Thread(target=route_thr,args=(target_lat,target_lon))


def fake_subscriber(msg):
        global current_lat,current_lon, target_lat, target_lon, current_speed

        raw = msg.data.split(",") 
        ## Command will be like described below 
        current_lat, current_lon, current_speed, target_lat, target_lon = raw

        route_thr(target_lat,target_lon)
                
#################################
### Monitor -> Autonom -> NAV ###
#################################




def ahrs_listener(msg):
        if msg is not None:
                raw = msg.data

                packets = raw.split(',')

                ahrs_val = packets[-1]
                
                ahrs_vals.append(ahrs_val)



def stop_listener(cmd):
        
        if cmd is 'NAV':
                rospy.signal_shutdown('Quit')



#######################################
########### INIT AND START ############
#######################################




rospy.init_node('Navigation',anonymous=True)

rospy.Subscriber(GPS_FIX_PUB_TOPIC, NavSatFix, callback_gps_fix)
rospy.Subscriber('/monitor/check_req', String, checker_send)
rospy.Subscriber('/automa/route', String, nav_task_handler)
rospy.Subscriber('/nav/direct', String, directRoute)
rospy.Subscriber('/stopper', String, stop_listener)
rospy.Subscriber('/nav/fakeListener', String, fake_subscriber)
rospy.Subscriber('/water/data', String, ahrs_listener)



rospy.spin()




#####################
#####  THREADS  #####
#####################


#Task handling is canceled here. Tasks are handled at automa node 

#tasks_handling = threading.Thread(target=task_handler)

distThread = threading.Thread(target=euclidean_dist)




####################
###### START #######
####################


# Early for that
####################
# distThread.start()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
        rate.sleep()
