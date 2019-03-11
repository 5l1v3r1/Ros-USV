#!/usr/bin/env python
# coding: utf8

import rospy
from std_msgs.msg import String
import threading
import time
from encoder import Encoder
from gps_common.msg import GPSFix
from gps_common.msg import GPSStatus
from sensor_msgs.msg import NavSatFix
from usb_finder import USB 

#############################



###Publishers###
rospy.loginfo('Started')
xbee_pub = rospy.Publisher('/comm/send_msg', String, queue_size=10)
check_req = rospy.Publisher('/monitor/check_req', String, queue_size=10)
stoper_pub = rospy.Publisher('/stopper', String, queue_size=10)
nav_pub = rospy.Publisher('/nav/directive', String, queue_size=10)
nav_mtr_fdbck = rospy.Publisher('/nav/motorFeedback', String, queue_size=10)
nav_gps_fdbck = rospy.Publisher('/nav/gpsFeedback', String, queue_size=10)
nav_direct = rospy.Publisher('/nav/direct', String, queue_size=10)
ahrs_pub = rospy.Publisher('/ahrs/velocity', String, queue_size=10)
auto_pub = rospy.Publisher('/auto/cmdlistener', String, queue_size=10)
nav_route_info = rospy.Publisher('/auto/routeinfo', String, queue_size=10)



GPS_FIX_PUB_TOPIC = 'fix'
GPS_EXT_FIX_PUB_TOPIC = 'extended_fix'

node_active = 0

lastDegree, lastRpm, lastDirection = 0 ,0 ,0
targetDegree, targetRpm, targetDirect = 0 ,0 ,0

def starter():
        msg = String()
        msg.data = '--Started--'
        xbee_pub.publish(msg)
        
        
        rate.sleep()

        msg_req = String()
        msg_req.data = 'Request'
        check_req.publish(msg_req)

        rate.sleep()

        start_sgnal = String()
        start_sgnal.data = 'Start'
        auto_pub.publish(start_sgnal)

        state_handler('Start')
        time.sleep(5)

        message_concat.start()
        


##############################################
########### Check everything is ok ###########
##############################################

def device_checker():
        devices = ['FTID', 'Garmin', 'Arduino']

        for device in devices:
                dev = USB.find(device)
                if dev is None:
                        rospy.loginfo('Fault with communicating ' + device)
                        rate.sleep()
                else:
                        msg = String()
                        msg.data = '--Started to communicate with devices--'
                        xbee_pub.publish(msg)
                        state_handler('Device-Check')
                        rate.sleep()

nodes = [
        'motor_control',
        'navigation',
        'water_sampler',
        'proximity',
        'comm',
        'autonom'
]


def check_nodes(msg):
        global node_active

        for node in nodes:
                if msg is node:
                        node_active = node_active +  1 

        if node_active is not 6:
                rospy.loginfo('There are lack nodes around')
                state_handler('Node-Check-Progress')
        else:
                rospy.loginfo('Everything is correct on nodes')
                state_handler('Node-Check')
                
                #####



## Values

water_vals = []
dist_vals = []
mtr_state_vals = []
gps_vals = []
state_vals = []
ahrs_vals = [0,0,0]

tasks = []
current_target = ''
current_task = ''
current_task_pri = 0



def water_sampler(msg):
        global water_vals

        if msg is not None:
                raw = msg.data

                packets = raw.split(',')

                ahrs_val = packets[-1]
                packets = packets[0:-1]
                
                water_vals.append(packets)
                ahrs_vals.append(ahrs_val)



def euclidean_calc(msg):
        global dist_vals
        raw = msg.data
        if msg is not None:
                dist_vals.append(raw)


def motor_state(msg):
        
        if msg is not None:
                raw = msg.data
                mtr_state_vals.append(raw)



def state_handler(state):
        if state is not None:
                state_vals.append('{}'.format(state))


def state_listener(msg):
        raw = msg.data
        state_vals.append(raw)                



def gps_listener(loc):
        lat = loc.latitude
        lon = loc.longitude
        mtime = time.time()
        position_co = loc.position_covariance

        gps_vals.append('{},{},{},{}'.format(lat,lon,mtime,position_co))


def message_concat():
        global state_vals
        global dist_vals
        global gps_vals
        global water_vals
        global mtr_state_vals


        

        msg = String()
        raw = state_vals[-1] + ',' + dist_vals[-1] + ',' + gps_vals[-1] + ',' +  water_vals[-1] + ',' + mtr_state_vals[-1] 
        msg.data = raw 
        xbee_pub.publish(msg)

        rate.sleep()


def motor_state_listener(msg):
        raw = msg.data
        mtr_state_vals.append(raw)



##For stoping components
#
# --MOTOR--
# motor_control
#
# --WATER SAMPLING--
# water_sampler
# 
# --Autonom--
# autonom


def emergency_listener(msg):
        cmd = String()

        if 'motor' in msg.data:
                cmd.data = 'motor_control,water_sampler,autonom'
                state_handler('Motor-Error')
                stoper_pub.publish(cmd)
                
                rate.sleep()
        elif 'water' in msg.data:
                cmd.data = 'water_sampler'
                state_handler('Sampler-Error')
                stoper_pub.publish(cmd)
                
                rate.sleep()
        elif 'gps' in msg.data:
                cmd.data = 'GPS'
                state_handler('GPS-Error')
                stoper_pub.publish(cmd)
                
                rate.sleep()

        rate.sleep()



def command_listener(msg):
        

        packets = msg.data.split(',')

        command = String()

        if packets[0] is 0:
                if packets[1] is 'all':
                        
                        command.data = 'All'
                        stoper_pub.publish(command)
                        
                        rate.sleep()
                        rospy.signal_shutdown('Quit')

                elif packets[1] is 'part':

                        if len(packets) is 3:
                                command.data = packets[2]
                                stoper_pub.publish(command)
                                
                                rate.sleep()
                                

        elif packets[0] is 1:

                if packets[1] is 'start':
                        starter()
                        rospy.loginfo('start')
                        device_checker()

        elif packets[0] is 2:

                if packets[1] is 'auto':
                        if len(packets) is 5:

                                direct_lat = packets[2]
                                direct_lon = packets[3]

                                msg = String()

                                        
                                msg.data = direct_lat + ',' + direct_lon

                                rospy.loginfo(direct_lat + ',' + direct_lon)

                                nav_direct.publish(msg)
                                

        elif packets[0] is 3:
                ## {3, lat , lon , wat}
                ##
                lat = packets[1]
                lon = packets[2]
                wat = packets[3]
                

                params = String()
                params.data = '{' + str(lat) + ',' + str(lon) + ',' + str(wat) + '}'
                rospy.loginfo('{' + str(lat) + ',' + str(lon) + ',' + str(wat) + '}')
                nav_route_info.publish(params)
                


                                
                                

#################################
### Monitor -> Autonom -> NAV ###
#################################

                                        

#####################################
#         Task Abreviations
#       NAV : Navigation task
#               ('NAV', lat, lon, priority)
#       MOTOR : Motor management task
#               ('MOTOR', speed, degree, priority)
#       WATER : Water sampling task
#               ('WATER', command, 'APPLY', priority)

           
                                
###Threads###


message_concat = threading.Thread(target=message_concat)

                                        

                                


#######################################
########### INIT AND START ############
#######################################





rospy.init_node('monitoring',anonymous=True)





rospy.Subscriber('/comm/received_msg', String, command_listener)
rospy.Subscriber('/water/data', String, water_sampler)
rospy.Subscriber('/monitor/check', String, check_nodes)
rospy.Subscriber('/nav/dist', String, euclidean_calc)
rospy.Subscriber('/motor/state', String, )
rospy.Subscriber('/emergency', String, emergency_listener)
rospy.Subscriber('/monitor/state',String, state_listener)
rospy.Subscriber(GPS_FIX_PUB_TOPIC, NavSatFix, gps_listener)


rospy.spin()



rate = rospy.Rate(10)
while not rospy.is_shutdown():
        rate.sleep()
