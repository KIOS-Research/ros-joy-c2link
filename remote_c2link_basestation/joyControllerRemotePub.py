#!/usr/bin/env python

# Author: Christos Georgiades
# Email: cgeorg15@ucy.ac.cy
# Date :07/07/2022

# license removed for brevity

import rospy
import threading
import time 
import math

from std_msgs.msg import String, Bool, Header
from dji_sdk.msg import ScanArea, MissionWaypoint, JoystickParams
from kios.msg import Telemetry, MissionDji, GpsInput, MissionCommandDJI

from dji_sdk.srv import SetJoystickMode, JoystickAction, ObtainControlAuthority
from sensor_msgs.msg import Joy

from geometry_msgs.msg import Vector3Stamped, Vector3

from math import sin, cos, atan2, sqrt
from simple_pid import PID



                    # r.sleep()


pidAlt = PID(Kp=1, Ki=0.0, Kd=0.2, sample_time=0.02,  output_limits=(-2.0, 5.0))
pidAlt.setpoint = 0     # value we are trying to achieve
pidAlt(0)               # value we read

pidDist = PID(Kp=0.35, Ki=0.00023, Kd=0.07, sample_time=0.02, output_limits=(-2.0, 8.0))
pidDist.setpoint = 0    # value we are trying to achieve
pidDist(0)              # value we read

printStep = 50
dronename = None

RAD_2_DEG = 57.29577951
DEG_2_RAD = 0.01745329252
EARTH_RADIUS = 6378137.0



historyJoystickAction = []
sampleIndex = 0
sampleSize = 10

sumJoystickAction = JoystickParams()
sumJoystickAction.x = 0.0
sumJoystickAction.y = 0.0
sumJoystickAction.z = 0.0
sumJoystickAction.yaw = 0.0

activeJoystickAction = JoystickParams()
activeJoystickAction.x = 0.0
activeJoystickAction.y = 0.0
activeJoystickAction.z = 0.0
activeJoystickAction.yaw = 0.0

for x in range(sampleSize):
    actionParams = JoystickParams()
    actionParams.x = 0.0
    actionParams.y = 0.0
    actionParams.z = 0.0
    actionParams.yaw = 0.0
    
    historyJoystickAction.append(actionParams)    


def setupJoystickMode():   
    joystickModeSrv=rospy.ServiceProxy(dronename+'/set_joystick_mode', SetJoystickMode)
      
    horizontal_mode = 1
    vertical_mode = 0
    yaw_mode = 1
    hoorizontal_coordinate = 1
    stable_mode = 1

    response = joystickModeSrv(horizontal_mode, vertical_mode, yaw_mode, hoorizontal_coordinate, stable_mode)
    print('Set Joystick Mode Response:', response)


def obtainControlAuthority(enable_obtain):
    #Type: dji_sdk/ObtainControlAuthority
    #Args: enable_obtain
    
    controlAuthoritySrv=rospy.ServiceProxy(dronename+'/obtain_release_control_authority', ObtainControlAuthority)
    response = controlAuthoritySrv(enable_obtain)
    
    print('Desired Control Authority:', enable_obtain, 'controlAuthoritySrv response:', response)


def joystickInputCB(joyInput):
    global droneLinkPub
    print("input",joyInput)
    droneLinkPub.publish(joyInput)


def publishAction(x, y, z, yaw):
    global activeJoystickAction, sumJoystickAction
    global historyJoystickAction, sampleIndex, sampleSize
    
    actionParams = JoystickParams()
    actionParams.x = x
    actionParams.y = y
    actionParams.z = z
    actionParams.yaw = yaw
    
    activeJoystickAction = actionParams
    
    # smooth input
#    sumJoystickAction.x -= historyJoystickAction[sampleIndex].x
#    sumJoystickAction.y -= historyJoystickAction[sampleIndex].y
#    sumJoystickAction.z -= historyJoystickAction[sampleIndex].z
#    sumJoystickAction.yaw -= historyJoystickAction[sampleIndex].yaw
#    
#    actionParams = JoystickParams()
#    actionParams.x = x
#    actionParams.y = y
#    actionParams.z = z
#    actionParams.yaw = yaw
#    
#    historyJoystickAction[sampleIndex] = actionParams
#    
#    if x * y * z * yaw == 0.0:
#        for i in range(sampleSize):
#            historyJoystickAction[i] = actionParams
#    
#    sumJoystickAction.x += x
#    sumJoystickAction.y += y
#    sumJoystickAction.z += z
#    sumJoystickAction.yaw += yaw
#    
#    sampleIndex += 1
#    if sampleIndex >= sampleSize:
#        sampleIndex = 0
#        
#    activeJoystickAction.x = sumJoystickAction.x / sampleSize
#    activeJoystickAction.y = sumJoystickAction.y / sampleSize
#    activeJoystickAction.z = sumJoystickAction.z / sampleSize
#    activeJoystickAction.yaw = sumJoystickAction.yaw / sampleSize


def pushAction():
    global activeJoystickAction
    
    response = joystickActionSrv(activeJoystickAction)
    if not response:
        print('================')
        print('joystickActionSrv response:', response)
        print(activeJoystickAction)
    
    #print('joystickActionSrv response:', response)
    


 
    
def listener(dji_name = "/matrice300"):
    global responsePub, droneStatePub, joystickActionSrv
    global dronename, r
    global droneLinkPub
    
    dronename = dji_name

    print('dronename:', dronename)
    rospy.init_node(dronename.replace('/','') + '_joy_publisher')
    r = rospy.Rate(25)  # 50hz

    print('Finding receiver joystick topic...')   
    uplinkNode = None
    uplinkTopic = None
    
    while not uplinkNode:
         #print('Finding receiver joystick topic...')   
         topics = rospy.get_published_topics()
 
         for topic in topics:
                 #print(topic)
                 if dronename not in topic[0]:
                     if 'matrice300_' in topic[0]:
                         uplinkNode = topic[0].split('/')[1]
                         uplinkTopic = '/' + uplinkNode + '/joy'
                         break
                     
         r.sleep()

# =============================================================================
#     uplinkTopic = None
#     while not uplinkTopic:
#         print('Finding receiver joystick topic...')   
#         topics = rospy.get_published_topics()
# 
#         for topic in topics:
#                 print(topic)
#                 if dronename not in topic[0]:
#                     if 'joy' in topic[0] and '/joy' != topic[0]:
#                         uplinkTopic = topic[0]
#                         break
#                     
#         r.sleep()
# =============================================================================

    print('\nSuccess\n\tuplinkTopic:\t' + str(uplinkTopic) + '\n')
    
    rospy.Subscriber('/joy', Joy, joystickInputCB)
    droneLinkPub = rospy.Publisher(uplinkTopic, Joy, queue_size=10)

    print('Joystick Controller publisher\tOK')
    rospy.spin()

if __name__=='__main__':
    print('Initializing remote Joystick Controller publisher...')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]

    dronename = '/matrice300' + '_' + boardId
    listener(dronename)
