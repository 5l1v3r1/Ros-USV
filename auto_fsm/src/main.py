#!/usr/bin/env python
# coding: utf8
import rospy
from std_msgs.msg import String
import threading
import time
import math
from usb_finder import USB
from encoder import Encoder
from gps_common.msg import GPSFix
from gps_common.msg import GPSStatus
from sensor_msgs.msg import NavSatFix

mon_pub = rospy.Publisher('/comm/send_msg', String, queue_size=10)
route_pub = rospy.Publisher('/automa/route', String, queue_size=10)
water_pub = rospy.Publisher('/water/command', String, queue_size=10)
check_pub = rospy.Publisher('/monitor/check', String, queue_size=10)
stoper_pub = rospy.Publisher('/stoper', String, queue_size=10)



GPS_FIX_PUB_TOPIC = 'fix'
GPS_EXT_FIX_PUB_TOPIC = 'extended_fix'

lat_vals = []
lon_vals = []
cov_vals = []
mtime_vals = []


waypoints = []
progress = 0
tasks = []

                
completed_tasks = []

# if devices connected, go on
def command_listener(cmd):
        raw = cmd.data
        #Messages should be like {1,2,3,4,5,6,7}
        #first value will indicate the command type but will not be investigated here 
        #it should be handled at monitoring node
        cmds = raw.split(',')
        if cmds[1] is 'Start':
                taskThrd.start()
        elif cmds[1] is 'Stop':
                taskThrd._stop()
                stop()


def start():
        global progress,waypoints
        

        while progress is not len(tasks):
                time.sleep(1)
                rospy.loginfo('Waiting for task')

        if progress < len(tasks):
                while progress is not len(waypoints):
                        
                        if is_completed(progress):

                                if progress is not 0:
                                        progress += 1

                                waypoints = waypoints[progress]

                                lat = float(waypoints[1])
                                lon = float(waypoints[2])
                                wat = bool(waypoints[3])

                                task_handler(lat, lon, wat)

                        else:
                                time.sleep(1)


def stop_listener(cmd):

        if cmd.data is 'NAV':
                rospy.signal_shutdown('Quit')


                        
#################################
### Monitor -> Autonom -> NAV ###
#################################

                        
def task_handler(lat, lon, wat):

        route = String()

        route.data = '{' + str(lat) + ',' + str(lon) + '}'
        route_pub.publish(route)

        while is_completed(progress):
                if wat:

                        water = String()
                        water.data = 'start'
                        water_pub.publish(water)

                        rospy.loginfo('Task Started')
                else:

                        water_stopper = String()
                        water.data = 'stop'
                        water_pub.publish(water_stopper)

                        rospy.loginfo('Task Stopped')
                        time.sleep(1)


                


                        
def get_completed(msg):
        global completed_tasks
        
        raw = int(msg.data)
        completed_tasks.append(raw)


def is_completed(taskNo):
        global completed_tasks

        i = 0
        while i is not len(completed_tasks):
                if taskNo is completed_tasks[i]:
                        return True
                else:
                        return False



def stop():
        msg = String()
        msg.data = 'All'
        stoper_pub.publish(msg)




def distance(h_lat, h_lon, t_lat, t_lon):
         
        R = 6373.0


        lat1 = math.radians(h_lat)
        lon1 = math.radians(h_lon)
        lat2 = math.radians(t_lat)
        lon2 = math.radians(t_lon)

        dlon = lon2 - lon1
        dlat = lat2 - lat1
                
        a = (math.sin(dlat/2))**2 + math.cos(lat1) * \
            math.cos(lat2) * (math.sin(dlon/2))**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist = R * c * 1000


        return dist




def checker_send(msg):
        if msg.data is 'Request':
                response = String()
                response.data = 'autonom'
                check_pub.publish(response)
                rate.sleep()




def route_listener(msg):
        raw = msg.data

        vals = raw.split(',')

        val_lat = vals[0]
        val_lon = vals[1]
        val_wat = vals[2]

        waypoints.append([val_lat, val_lon, val_wat])





def gps_listener(loc):
        lat = loc.latitude
        lon = loc.longitude
        mtime = time.time()
        position_co = loc.position_covariance

        lat_vals.append(lat)
        lon_vals.append(lon)
        mtime_vals.append(mtime)
        cov_vals.append(position_co)












#########################################
#               COMMANDS                ###
#                                       ###
# ------                                ###
# Motor:                                ###
# ------                                ###
# {transmission,angle,revertorforward}  ###
#                                       ###
# -----------                           ###
# NAVIGATION and MONITORING:            ###
# -----------                           ###
###########################################
#               Commands                  #
#               --------                  #
# first index should include command type #
# '0' is emergency stop                   #
# '1' is start command                    #
# '2' is direction indicator              #
# '3' is navigation parameters            #
#                                         #
#            Start telemetry              #
# {1,'Start'}                             #
# This will start boat                    #
# Before the start send navigation data   #
#                                         #
#         Navigation parameters           #
#                                         #
#         *{3,'add', lat, lon}            #
#                                         #
#       -Direction parameters             #
#         *{2,'auto', lat,lon, priority}  #
#          -Priority Levels               #
#          * '0' : None                   #
#          * '1' : first to proceed       #
#          * '2' : after the current task #
#          * '3' : wait for the program   #
#                                         #
#            Stop COMMAND                 #
#                                         #
# For stopping system parts               #
# {0, 'part', 'motor'}                    #
# {0, 'all'}                              #
###########################################
#                                       ###
# ---------------                       ###
# WATER SAMPLING:                       ###
# ---------------                       ###
# {start} or {stop}                     ###
#                                       ###
###########################################





#######################################
########### INIT AND START ############
#######################################


rospy.init_node('autonom', anonymous=True)


###################
##### THREADS #####
###################


taskThrd = threading.Thread(target=start)



###################
#####  Nodes  #####
###################





rospy.Subscriber('/auto/cmdlistener', String, command_listener)
rospy.Subscriber('/monitor/check_req', String, checker_send)
rospy.Subscriber(GPS_FIX_PUB_TOPIC, NavSatFix, gps_listener)
rospy.Subscriber('/auto/routeinfo', String, route_listener)
rospy.Subscriber('/stopper', String, stop_listener)





###################
###### START ######
###################


rospy.spin()


rate = rospy.Rate(10)
while not rospy.is_shutdown():
        rate.sleep()

