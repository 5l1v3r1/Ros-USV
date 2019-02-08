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

mon_pub = rospy.Publisher("/comm/send_msg", String, queue_size=10)
route_pub = rospy.Publisher("/automa/route", String, queue_size=10)
water_pub = rospy.Publisher("/water/command", String, queue_size=10)
check_pub = rospy.Publisher("/monitor/check", String, queue_size=10)
stoper_pub = rospy.Publisher("/stoper", String, queue_size=10)



GPS_FIX_PUB_TOPIC = "fix"
GPS_EXT_FIX_PUB_TOPIC = "extended_fix"

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
        raw = Encoder(cmd).decode()
        #Messages should be like {1,2,3,4,5,6,7}
        #first value will indicate the command type but will not be investigated here 
        #it should be handled at monitoring node
        cmds = raw.split(",")
        if cmds[1] is "Start":
                taskThrd.start()
        elif cmds[1] is "Stop":
                taskThrd._stop()
                stop()


def start():
        global progress
        

        while progress is not len(tasks):
                time.sleep(1)
                rospy.loginfo("Waiting for task")

        if progress is not len(tasks):
                while progress is not len(tasks):
                        task = progress
                        if is_completed(progress):
                                progress += 1
                                task = tasks[progress]

                                lat = float(task[1])
                                lon = float(task[2])
                                wat = float(task[3])

                                task_handler(lat, lon, wat)

                        else:
                                time.sleep(1)
                        
#################################
### Monitor -> Autonom -> NAV ###
#################################

                        
def task_handler(lat, lon, wat):

        route = String()

        route.data = "{" + str(lat) + "," + str(lon) + "}"
        route_pub.publish(route)

        if wat:
                
        


                        
def get_completed(msg):
        global completed
        
        raw = int(msg)
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
        msg.data = "All"
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
        if msg is "Request":
                response = String()
                response.data = "autonom"
                check_pub.publish(response)
                rate.sleep()


def gps_listener(loc):
        lat = loc.latitude
        lon = loc.longitude
        mtime = time.time()
        position_co = loc.position_covariance

        lat_vals.append(lat)
        lon_vals.append(lon)
        mtime_vals.append(mtime)
        cov_vals.append(position_co)


rospy.Subscriber("/auto/cmdlistener", String, command_listener)
rospy.Subscriber("/monitor/check_req", String, checker_send)
rospy.Subscriber(GPS_FIX_PUB_TOPIC, NavSatFix, gps_listener)


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
# "0" is emergency stop                   #
# "1" is start command                    #
# "2" is direction indicator              #
# "3" is navigation parameters            #
#                                         #
#            Start telemetry              #
# {1,"Start"}                             #
# This will start boat                    #
# Before the start send navigation data   #
#                                         #
#         Navigation parameters           #
#                                         #
#         *{3,"add", lat, lon}            #
#                                         #
#       -Direction parameters             #
#         *{2,"auto", lat,lon, priority}  #
#          -Priority Levels               #
#          * "0" : None                   #
#          * "1" : first to proceed       #
#          * "2" : after the current task #
#          * "3" : wait for the program   #
#                                         #
#            Stop COMMAND                 #
#                                         #
# For stopping system parts               #
# {0, "part", "motor"}                    #
# {0, "all"}                              #
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

rospy.init_node("autonom", anonymous=True)


###################
##### THREADS #####
###################


taskThrd = threading.Thread(target=start)



###################
###### START ######
###################


rospy.spin()


rate = rospy.Rate(10)
while not rospy.is_shutdown():
        rate.sleep()

