#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import signal
import subprocess
import datetime
import socket

import rospy

import numpy as np
import math

#from scipy.stats import norm
#from scipy.optimize import leastsq,least_squares
#from ekf_drps import EKF

from dji_sdk.msg import telemetry2, Packet, Packet2, Gimbal
from dji_sdk.msg import EscData, ESCStatusIndividual
from dji_sdk.srv import GetSingleBatteryDynamicInfo

from kios.msg import Telemetry, TerminalHardware, DroneHardware
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped

from multimaster_msgs_fkie.msg import LinkStatesStamped


from peewee import * 

file_path = os.path.realpath(__file__)
file_path = file_path.split('DRPS', 1)[0] + 'src/dji_sdk/scripts/'
print(file_path)

#/home/kios/Documents/multimaster_db_matrice300_q3_demo/DRPS/ekffusion_drps_no_hackrf.py
sys.path.append(file_path)


def GetBoardID():
    f = open("/var/lib/dbus/machine-id", "r")
    boardID = f.read().strip()[0:4]
    #print(boardID)
    return boardID



boardId = GetBoardID()
dronename = '/matrice300' + '_' + boardId

print(dronename)

#dronename2='/matrice210RTK'
lat= 0 #import sensor data from ros 
lon= 0 
rawLat = 0
rawLon = 0
angularx=0
angulary=0
linearaccellxy=0
headingangle=0
gpsdat=np.zeros((2,2))
velocitylinear=0
altitude=0
dataset=np.zeros((699,25))
#def velocitycallback(messagedata):

# Drone
d_quat_x = 0
d_quat_y = 0
d_quat_z = 0
d_quat_w = 0
# Gimbal
g_yaw = 0
g_pitch = 0
g_roll = 0

meters_sum, x_sum, y_sum = 0, 0, 0
RPSarray2=[0,0]


batteryPercentage = 0
batteryVoltage = 0
batteryCurrent = 0
batteryCapacity = 0
def batteryCB(batteryStatus):
    global batteryPercentage, batteryVoltage, batteryCurrent, batteryCapacity   
    batteryPercentage = batteryStatus.percentage
    batteryVoltage = batteryStatus.voltage
    batteryCurrent = batteryStatus.current
    batteryCapacity = batteryStatus.capacity


escCount = 0
escSpeed = []
escVoltage = []
escTemperature = []
def escMotorCB(escMotorStatus):
    global escCount, escSpeed, escVoltage, escTemperature
    #/matrice300_5807/ESC_data
    escCount = 0
    escSpeed = []
    escVoltage = []
    escTemperature = []
    
    for escMotor in escMotorStatus.esc:
        if escMotor.speed > 0:
            escCount += 1
            escSpeed.append(escMotor.speed)
            escVoltage.append(escMotor.voltage)
            escTemperature.append(escMotor.temperature)
    

gpssig = 0
satnumber = 0

#sequence number to count packet loss
seq = 0
global packlosspub, droneState, droneIDpub
droneState = "On Ground"
droneIdAck = False
droneIDpub = None


hostname = hostname = socket.getfqdn()
local_ip = socket.gethostbyname_ex(hostname)[2][0]
platform_ip = None
# print("Hostname:", hostname, "Local IP:", local_ip)


db = SqliteDatabase('drone.db')
class Data(Model):
    id = AutoField()

    seq = IntegerField()
    serialVersionUID = TextField()

    head = FloatField()
    vel = FloatField()
    lon = FloatField()
    lat = FloatField()
    altitude = FloatField()

    date_time = TextField()
    rostime_secs = IntegerField()
    fl_time = TextField()

    battery = IntegerField()
    batteryThresh = IntegerField()

    cpuTemp = IntegerField()
    gpuTemp = IntegerField()
   
    gpssgnal = IntegerField()
    gpssat = IntegerField()
    homLat = FloatField()
    homLon = FloatField()

    d_quat_x = FloatField()
    d_quat_y = FloatField()
    d_quat_z = FloatField()
    d_quat_w = FloatField()

    g_yaw = FloatField()
    g_pitch = FloatField()
    g_roll = FloatField()

    class Meta:
        database = db
        
class TelemetryData(Model):
    id = AutoField()

    seq = IntegerField()
    serialVersionUID = TextField()

    rostime_secs = IntegerField()
    rostime_nsecs = IntegerField()
    flightTimeSecs = IntegerField()
    
    batteryThresh = IntegerField()
    batteryPercentage = IntegerField()
    
    gpsSignal = IntegerField()  
    satelliteNumber = IntegerField()
    
    altitude = IntegerField()
    heading = FloatField()
    velocity = FloatField()
    longitude = FloatField()
    latitude = FloatField()
    
    homeLatitude = FloatField()
    homeLongitude = FloatField()

    droneState = TextField()
    gimbalAngle = FloatField()

    class Meta:
        database = db
        
class TerminalHardwareData(Model):
    id = AutoField()

    seq = IntegerField()
    serialVersionUID = TextField()

    ram_use = IntegerField()
    ram_max = IntegerField()

    swap_use = IntegerField()
    swap_max = IntegerField()
    swap_cache = IntegerField()
    
    emc_usage = IntegerField()  
    
    cpu_core_count = IntegerField()
    cpu_core_usage = IntegerField()
    cpu_core_freq = IntegerField()
    cpuTemp = IntegerField()
    
    gr3d_usage = IntegerField()
    gr3d_freq = IntegerField() 
    gpuTemp = IntegerField()

    class Meta:
        database = db


class DroneHardwareData(Model):
    id = AutoField()

    seq = IntegerField()
    serialVersionUID = TextField()

    batteryThreshold = IntegerField()
    batteryPercentage = IntegerField()
    batteryPercentage = IntegerField()
    
    batteryVoltage = IntegerField()
    batteryCurrent = IntegerField()   
    batteryCapacity = IntegerField()  
    
    escCount = IntegerField()    
    escSpeed = IntegerField()
    escVoltage = FloatField()
    escTemperature = FloatField()


    class Meta:
        database = db


def signal_handler(sig, frame): 
    global droneIDpub
    hello_msg = "{\"Timestamp\":\""+str(rospy.get_rostime())+"\",\"DroneName\":\""+dronename.replace('/','')+"\",\"Connected\":\"False\",\"Model\":\"MATRICE_300_RTK\",\"cameraVideoStreamSource\":\"unknown\",\"cameraName\":\"Zenmuse_H20T\"}"
    droneIDpub.publish(hello_msg)
    oc.rtmpHandler.deinitStream()
    print('Connected: False sent to platform.')
    sys.exit(0)


def EKF2_callback(rpsdata2):
    global RPSarray2
    RPSarray2  =  rpsdata2.data


def vision_callback(flowdata):
    global meters_sum, x_sum, y_sum
    meters_sum, x_sum, y_sum = flowdata.data
    print("DRPS Vision data: ",meters_sum, x_sum, y_sum,flowdata.data)
    
def imucallback(messagedat):
    global angularx,angulary,linearaccellxy
    angularx=messagedat.angular_velocity.x
    angulary=messagedat.angular_velocity.y
    linearaccellxy=math.sqrt((messagedat.linear_acceleration.x)**2 + (messagedat.linear_acceleration.y)**2)

def telemetrycallback(teldat):
    global lat,lon,headingangle,velocitylinear,altitude,rostime,rostime_nano,fligth_time_seconds, batteryPercentage, gpssig, satnumber, home_lat, home_lon, drone_serial
    lat=teldat.latitude
    lon=teldat.longitude
    headingangle=teldat.heading
    velocitylinear=teldat.velocity
    altitude=teldat.altitude
    rostime=teldat.rostime_secs
    rostime_nano=teldat.rostime_nsecs
    fligth_time_seconds=teldat.flightTimeSecs
    batteryPercentage=teldat.batteryPercentage
    gpssig=teldat.gpsSignal
    satnumber=teldat.satelliteNumber
    home_lat=teldat.homeLatitude
    home_lon=teldat.homeLongitude
    drone_serial=teldat.serialVersionUID
    
    '''
    DDdata=Telemetry()
    
    DDdata.rostime_secs = 0
    DDdata.rostime_nsecs = 0
    DDdata.flightTimeSecs = 0
    
    DDdata.batteryThreashold = 0
    DDdata.batteryPercentage = 0
    
    DDdata.gpsSignal = 0
    DDdata.satelliteNumber = 0
    
    DDdata.altitude=altitude
    DDdata.latitude=lat
    DDdata.longitude=lon
    DDdata.heading=headingangle
    
    DDdata.velocity = 0.0
    
    DDdata.homeLatitude = lat
    DDdata.homeLongitude = lon
    
    DDdata.droneState = "Flying"
    DDdata.gimbalAngle = 0.0
    DDdata.serialVersionUID = 100
    
    rpsddPub.publish(DDdata)'''


def telemetrycallback_b(teldat2):
    global lat2,lon2,headingangle2,velocitylinear2,altitude2
    lat2=teldat2.latitude
    lon2=teldat2.longitude
    headingangle2=teldat2.heading
    velocitylinear2=teldat2.velocity
    altitude2=teldat2.altitude

def rawgpscallback(raw):
    global rawLat, rawLon
    rawLat = raw.latitude
    rawLon = raw.longitude

def packetlosscb(data):
        global packlosspub
        print('ta start ine', data.start)
        db.create_tables([Data])
        print('print to for',data.fin-data.start)
        for i in range(data.fin-data.start,data.fin):
            print('to fin ine',data.fin)
            print('to seq ine',seq)
            print('to i ine',i)
            for test in Data.select().where(Data.seq == i):
                print('to test ine :',test)
                newpacket2 = Packet2()
                newpacket2.seq = test.seq
                newpacket2.serialVersionUID = test.serialVersionUID.encode('ascii', 'replace')

                print('to xameno packet ine:',newpacket2.seq)
                newpacket2.heading = test.head
                newpacket2.velocity = test.vel
                newpacket2.longitude = test.lon
                newpacket2.latitude = test.lat
                newpacket2.altitude = test.altitude
                newpacket2.date_time = test.date_time.encode('ascii', 'replace')
                newpacket2.rostime_secs = 0
                newpacket2.flightTimeSecs = int(test.fl_time)
        
                newpacket2.batteryThreshold = test.batteryThresh
                newpacket2.batteryPercentage = test.battery
                newpacket2.cpuTemp = test.cpuTemp
                newpacket2.gpuTemp = test.gpuTemp

                newpacket2.gpsSignal = test.gpssgnal
                newpacket2.satelliteNumber = test.gpssat
                newpacket2.homeLatitude = test.homLat
                newpacket2.homeLongitude = test.homLon
                   
                       
                # Drone Orientation
                newpacket2.d_quat_x = test.d_quat_x
                newpacket2.d_quat_y = test.d_quat_y
                newpacket2.d_quat_z = test.d_quat_z
                newpacket2.d_quat_w = test.d_quat_w
                # Gimbal Orientation
                newpacket2.g_yaw = test.g_yaw
                newpacket2.g_pitch = test.g_pitch
                newpacket2.g_roll = test.g_roll
                packlosspub.publish(newpacket2)


def DroneOrientationCB(msg):
    global d_quat_x, d_quat_y, d_quat_z, d_quat_w
    d_quat_x = msg.quaternion.x
    d_quat_y = msg.quaternion.y
    d_quat_z = msg.quaternion.z
    d_quat_w = msg.quaternion.w 


def Gimbal_cb(gimbal):
    global g_yaw, g_pitch, g_roll
    g_yaw = gimbal.vector.z
    g_pitch = gimbal.vector.y
    g_roll = gimbal.vector.x


def GetJetsonCPUTemp():
    global cpuTemp
    f = open("/sys/class/thermal/thermal_zone0/temp", "r")
    cpuTemp = int(f.read().strip())
    return cpuTemp

def GetJetsonGPUTemp():
    f = open("/sys/class/thermal/thermal_zone1/temp", "r")
    gpuTemp = int(f.read().strip())
    return gpuTemp

# tegrastats
# RAM 2130/7774MB (lfb 1004x4MB) SWAP 0/3887MB (cached 0MB) 
# CPU [11%@1190,2%@1190,1%@1190,1%@1190,1%@1190,2%@1190] EMC_FREQ 0% GR3D_FREQ 0% 
# AO@31C GPU@31.5C PMIC@100C AUX@31.5C CPU@33C thermal@31.95C

# sudo tegrastats
# RAM 2387/7774MB (lfb 961x4MB) SWAP 0/3887MB (cached 0MB) 
# CPU [14%@1190,9%@1190,2%@1190,4%@1190,0%@1190,1%@1344] EMC_FREQ 1%@1600 GR3D_FREQ 0%@114 
# VIC_FREQ 0%@115 APE 150 MTS fg 0% bg 3% 
# AO@31C GPU@31.5C PMIC@100C AUX@31.5C CPU@33C thermal@31.95C 
# VDD_IN 3528/3569 VDD_CPU_GPU_CV 410/426 VDD_SOC 1107/1115
def GetTerminalHardware():     
    #p = subprocess.Popen('tegrastats | head -n1', stdout=subprocess.PIPE, shell = True)
    p = subprocess.Popen(('tegrastats'), stdout=subprocess.PIPE)
    tegrastats_stdout = subprocess.check_output(('head', '-n1'), stdin=p.stdout)
     
    #tegrastats_stdout = p.stdout.readline()
    tegrastats_stdout = tegrastats_stdout.split()
    
    p.kill()    
    # print(tegrastats_stdout)

    termHwPacket = TerminalHardware()
    termHwPacket.seq = seq
    termHwPacket.uid = GetBoardID()  
    
    ram_str = tegrastats_stdout[1]
    #print(ram_str)
    ram_str = ram_str.replace('MB', '').split('/')
    #print(ram_str)
    
    termHwPacket.ram_use = int(ram_str[0])
    termHwPacket.ram_max = int(ram_str[1])
    
    swap_str = tegrastats_stdout[5]
    #print("swap_str", swap_str)
    swap_str = swap_str.replace('MB', '').split('/')
    #print("swap_str", swap_str)
    termHwPacket.swap_use = int(swap_str[0])
    termHwPacket.swap_max = int(swap_str[1])
    
    cached_swap_str = tegrastats_stdout[7]
    #print(cached_swap_str)
    cached_swap_str = cached_swap_str.replace('MB)', '')
    
    termHwPacket.swap_cache = int(cached_swap_str)
    
    emc_str = tegrastats_stdout[11]
    #print(emc_str)
    termHwPacket.emc_usage = int(emc_str.replace('%', ''))
    
    cpu_str = tegrastats_stdout[9] 
    #print(cpu_str)
    cpu_str = cpu_str.replace('[','').replace(']','')    
    cpu_str = cpu_str.split(',')
    
    cpu_core_usage = []
    cpu_core_freq = []
    for core in cpu_str:
        core_str = core.split('%@')
        cpu_core_usage.append(int(core_str[0]))
        cpu_core_freq.append(int(core_str[1]))
        
    termHwPacket.cpu_core_count = len(cpu_core_usage)
        
    termHwPacket.cpu_core_usage = cpu_core_usage
    termHwPacket.cpu_core_freq = cpu_core_freq
    
# =============================================================================
#     cpu_str_0 = cpu_str[0].split('%@')
#     #print(cpu_str_0)
#     termHwPacket.cpu_core0_usage = int(cpu_str_0[0])
#     termHwPacket.cpu_core0_freq = int(cpu_str_0[1])
#     
#     cpu_str_1 = cpu_str[1].split('%@')
#     termHwPacket.cpu_core1_usage = int(cpu_str_1[0])
#     termHwPacket.cpu_core1_freq = int(cpu_str_1[1])
#     
#     cpu_str_2 = cpu_str[2].split('%@')
#     termHwPacket.cpu_core2_usage = int(cpu_str_2[0])
#     termHwPacket.cpu_core2_freq = int(cpu_str_2[1])
#     
#     cpu_str_3 = cpu_str[3].split('%@')
#     termHwPacket.cpu_core3_usage = int(cpu_str_3[0])
#     termHwPacket.cpu_core3_freq = int(cpu_str_3[1])
#     
#     cpu_str_4 = cpu_str[4].split('%@')
#     termHwPacket.cpu_core4_usage = int(cpu_str_4[0])
#     termHwPacket.cpu_core4_freq = int(cpu_str_4[1])
#     
#     cpu_str_5 = cpu_str[5].split('%@')
#     termHwPacket.cpu_core5_usage = int(cpu_str_5[0])
#     termHwPacket.cpu_core5_freq = int(cpu_str_5[1])
# =============================================================================
    termHwPacket.cpuTemp = GetJetsonCPUTemp()
    
    gr3d_str = tegrastats_stdout[13]
    termHwPacket.gr3d_usage = int(gr3d_str.replace('%',''))
    
    #    gpu.0/devfreq/17000000.gv11b/cur_freq
    f = open("/sys/devices/gpu.0/devfreq/17000000.gv11b/cur_freq", "r")
    termHwPacket.gr3d_freq = int(f.readline()) / 1000000
    termHwPacket.gpuTemp = GetJetsonGPUTemp()
    f.close()
    
    return termHwPacket

def Create3dMesh():
    meshConstructorDir = '/home/jetson/Documents/mesh_construction/aidersplatform_algorithms_mesh_construction.py'
    
    # Move Some Files
    p = subprocess.Popen(('python', meshConstructorDir), stdout=subprocess.PIPE)
    p.wait()
    p.kill()
    
    print('Mesh Construction has finished')
    print('Posting 3d Mesh to platform')
    
    # Move some more
    
    # Post 3d mesh to platform
    
    
def DroneIdAckCB(ack):
    global droneIdAck
    
    print('Ack received', ack)
    
    if ack.data == '1':
        droneIdAck = True
                  
            
def checkPortStatus(ip, port):
    p = subprocess.Popen(("nc", "-zvw5", ip, port), stderr = subprocess.PIPE)
    status = p.communicate()[1]
    
    print(str(status))
    
    
    # print (rc)
    # print (status)
    if 'succeeded' in str(status):
        return True
    else:
        return False
    
    
try: 
    print('try empike')
    print('try empike exiting ;)')
    
#    rospy.init_node('ekffusionnode' + '_' + GetBoardID())
#    #rospy.init_node('ekffusionnode',anonymous=False)
#
#    
#
#    #velsub=rospy.Subscriber(dronename+'/velocity',Vector3Stamped,velocitycallback)
#    
#    
#    #telemetrysub=rospy.Subscriber(dronename+'/telemetry2',telemetry2,telemetrycallback)   
#    #rpsddPub=rospy.Publisher(dronename+'/Telemetry',Telemetry, queue_size=1)
#    #telemetrysub2=rospy.Subscriber(dronename2+'/Telemetry',Telemetry,telemetrycallback_b)
#    
#    
#    #ekfPub=rospy.Publisher(dronename+'/EKF',Float64MultiArray, queue_size=1) # need to create a new publisher for the jamming agent 
#    #ekfPub_jam=rospy.Publisher(dronename+'/EKFjam',Float64MultiArray, queue_size=1) # need to create a new publisher for the jamming agent 
#    #ekfSub=rospy.Subscriber(dronename2+'/EKF',Float64MultiArray, EKF2_callback)
#    
#
#    #hackrfsub=rospy.Subscriber(dronename+'/hackrf',hackrf,hackrfcallback)
#    #visionsub=rospy.Subscriber(dronename+'vis/visiondata',Float64MultiArray,vision_callback)
#
#    # Dikes Mou Pipes
#    droneIDpub = rospy.Publisher('/droneIds',String, queue_size=2)  
#    imuSub=rospy.Subscriber(dronename+'/imu',Imu,imucallback)
#    
#    
#    rawGpsSub=rospy.Subscriber(dronename+'/raw_gps_position',NavSatFix,rawgpscallback)
#    gpsPub=rospy.Publisher(dronename+'/GPSmeters',Float64MultiArray, queue_size=1)
#    
#    packlosspub=rospy.Publisher(dronename+'/PacketLoss',Packet2, queue_size=1)
#    packetSub=rospy.Subscriber(dronename+'/Packet',Packet, packetlosscb)
#    packetPub2=rospy.Publisher(dronename+'/Packet2',Packet2, queue_size=1)
#    
#    droneOrientationSub = rospy.Subscriber(dronename+'/attitude',QuaternionStamped, DroneOrientationCB)
#    gimbalSub=rospy.Subscriber(dronename+'/gimbal_angle',Vector3Stamped, Gimbal_cb)
#
#    terminalHwPub = rospy.Publisher(dronename+'/TerminalHardware',TerminalHardware, queue_size=1)
#    
#    print('Finding platform Ip...')
#    linkstatsSub = rospy.Subscriber('/master_discovery/linkstats', LinkStatesStamped, findPlatformIp)
#    #while platform_ip is None:
#        
#        #findPlatformIp(linkstats)
#    
#    #rosDevicesSub = rospy.Subscriber('/master_discovery/linkstats', LinkStatesStamped, connectedIpCB)
#    
#    for x in range(10):
#        print(x)
#        if platform_ip:
#            break
#        
#        rospy.sleep(1)
#    
#    print('Setting up Mission Middleware')
#    import telemetryTranslator as  tt
#    import csv2rosMission as mm
#    import orthocamera as oc
#    
#    # Setup Mission Middleware
## =============================================================================
##     tt.listener(dji_name=dronename)
##     print('telemetryTranslator:\tReady')
##     
##     mm.listener(dji_name=dronename)
##     print('csv2rosMission:\tReady')
##     
##     oc.listener(platform_ip, dji_name=dronename)
##     print('orthocamera:\tReady')
##     
##     print('Middleware Setup:\tREADY')
## 
##     # jetsonThermalsPub=rospy.Publisher(dronename+'/JetsonThermals',JetsonThermals,queue_size=1)
##     
## 
##     
##     #while(np.count_nonzero(dataset)<=0):
##         #rate.sleep()
##     print('Setting up Mission Middleware')
##     import telemetryTranslator as  tt
##     import csv2rosMission as mm
##     import orthocamera as oc
##     
##     # Setup Mission Middleware
##     tt.listener(dji_name=dronename)
##     print('telemetryTranslator:\tReady')
##     
##     mm.listener(dji_name=dronename)
##     print('csv2rosMission:\tReady')
##     
##     oc.listener(platform_ip, dji_name=dronename)
##     print('orthocamera:\tReady')
##     
##     print('Middleware Setup:\tREADY')
## =============================================================================
#
#
#    rate=rospy.Rate(1)
#    precision = 8
#    np.set_printoptions(precision = precision)
#
#    db.connect()
#
#    home_lat = lat
#    home_lon = lon
#    
#    droneIdAckSub = rospy.Subscriber(dronename+'/handshake',String, DroneIdAckCB)
#    
#    signal.signal(signal.SIGINT, signal_handler)
#
#    #rospy.sleep(2)
#    hello_msg = "{\"Timestamp\":\""+str(rospy.get_rostime())+"\",\"DroneName\":\""+dronename.replace('/','')+"\",\"Connected\":\"True\",\"Model\":\"MATRICE_300_RTK\",\"cameraVideoStreamSource\":\"unknown\",\"cameraName\":\"Zenmuse_H20T\"}"
#    timeout = 1000000
#    print(hello_msg)
#    for i in range(0, timeout):
#        droneIDpub.publish(hello_msg)
#        print('Attempt:\t' + str(i))
#        rospy.sleep(5)
#        
#        if droneIdAck:
#            print('Platform register Successful')
#            break
# 
#    if not droneIdAck:
#        print('Could not register on platform.\n')      
#        
#    while not rospy.is_shutdown():
#        now = datetime.datetime.now()
#        timestmp = now.strftime("%Y%m%d-%H:%M:%S.%s")
#        ekfData = Float64MultiArray()
#        ekfData.data = [0.55,0.56]
#        #ekfPub.publish(ekfData) # ekf+ the vision data xposition yposition offset
#                    
#        #plt.legend()
#        #plt.pause(0.05)
#        rate.sleep()
#        gpsData = Float64MultiArray()
#        gpsData.data = [0.23,23.0]
#        gpsPub.publish(gpsData)
#        gpsdat[0][0]=23.0
#        gpsdat[0][1]=0.23
#        rpsjamdat=Float64MultiArray()
#        rpsjamdat.data= [0.255,0.233]
#        #ekfPub_jam.publish(rpsjamdat) # new publisher 
#        
#        rostime=timestmp
#        #publish Telemetry data to db
#        newpacket2 = Packet2()
#        newpacket2.seq = seq
#        newpacket2.serialVersionUID = GetBoardID()
#
#        newpacket2.heading = headingangle
#        newpacket2.velocity = velocitylinear
#        newpacket2.longitude = lon
#        newpacket2.latitude = lat
#        newpacket2.altitude = altitude
#        
#        newpacket2.date_time = rostime
#        #newpacket2.rostime_secs = datetime.datetime.timestamp()
#        newpacket2.flightTimeSecs = 0
#                    
#        newpacket2.batteryThreshold = 0
#        newpacket2.batteryPercentage = batteryPercentage
#        newpacket2.cpuTemp = GetJetsonCPUTemp()
#        newpacket2.gpuTemp = GetJetsonGPUTemp()
#        
#        newpacket2.gpsSignal = gpssig
#        newpacket2.satelliteNumber = satnumber
#        newpacket2.homeLatitude = home_lat
#        newpacket2.homeLongitude = home_lon
#                       
#        # Drone Orientation
#        newpacket2.d_quat_x = d_quat_x
#        newpacket2.d_quat_y = d_quat_y
#        newpacket2.d_quat_z = d_quat_z
#        newpacket2.d_quat_w = d_quat_w
#        # Gimbal Orientation
#        newpacket2.g_yaw = g_yaw
#        newpacket2.g_pitch = g_pitch
#        newpacket2.g_roll = g_roll
#        
#        termHW = GetTerminalHardware()
#
#        packetPub2.publish(newpacket2)
#        terminalHwPub.publish(termHW)
#        
#        if seq % 100 == 0:
#            print(newpacket2.serialVersionUID, newpacket2.date_time)
#                        
#        db.create_tables([Data])
#        Data.create(seq=newpacket2.seq, serialVersionUID = newpacket2.serialVersionUID,
#                    head=newpacket2.heading, vel=newpacket2.velocity, lon=newpacket2.longitude, lat=newpacket2.latitude, altitude=newpacket2.altitude,
#                    date_time = newpacket2.date_time, rostime_secs = 0, fl_time=newpacket2.flightTimeSecs,
#                    battery=newpacket2.batteryPercentage, batteryThresh = newpacket2.batteryThreshold,
#                    cpuTemp = newpacket2.cpuTemp, gpuTemp = newpacket2.gpuTemp,
#                    gpssgnal=newpacket2.gpsSignal, gpssat=newpacket2.satelliteNumber, homLat = newpacket2.homeLatitude, homLon = newpacket2.homeLongitude,
#                    d_quat_x = newpacket2.d_quat_x, d_quat_y = newpacket2.d_quat_y ,d_quat_z = newpacket2.d_quat_z,d_quat_w = newpacket2.d_quat_w,
#                    g_yaw= newpacket2.g_yaw ,g_pitch = newpacket2.g_pitch, g_roll=newpacket2.g_roll
#                    )
#
#
#        seq = seq+1
#                        
#        db.close()
        
        
        

except rospy.ROSInterruptException:
    pass


def findPlatformIp():
    print('Finding platform Ip...')
    platformIp = None

    while not platformIp:
        linkStats = rospy.wait_for_message('/master_discovery/linkstats', LinkStatesStamped)
        rosdevices = linkStats.links
        
        for device in rosdevices:
            if device.destination != local_ip:
                if checkPortStatus(device.destination, '8000') and checkPortStatus(device.destination, '1935'):
                    platformIp = device.destination
                    print("Platform Ip:", device)
                    return platformIp

        rospy.sleep(1)


def handshakePlatform():
    global droneIdPub, droneIdAckSub, droneIdAck
    
    droneIdAckSub = rospy.Subscriber(dronename+'/handshake',String, DroneIdAckCB)
    droneIdPub = rospy.Publisher('/droneIds',String, queue_size=2)  
    
    
    hello_msg = "{\"Timestamp\":\""+str(rospy.get_rostime())+"\",\"DroneName\":\""+dronename.replace('/','')+"\",\"Connected\":\"True\",\"Model\":\"MATRICE_300_RTK\",\"cameraVideoStreamSource\":\"unknown\",\"cameraName\":\"Zenmuse_H20T\"}"
    timeout = 1000000
    print(hello_msg)
    for i in range(0, timeout):
        droneIdPub.publish(hello_msg)
        print('Attempt:\t' + str(i))
        rospy.sleep(5)
        
        if droneIdAck:
            print('Platform register Successful')
            droneIdAck = False
            return True
 
    if not droneIdAck:
        print('Could not register on platform.\n') 
        return False


def setupTopics():
    global imuSub, rawGpsSub, gpsPub, droneOrientationSub, gimbalSub, terminalHwPub, droneHwPub
    global packlosspub, packetSub, packetPub2
    
    print('Setting up rostopics...')
    imuSub=rospy.Subscriber(dronename+'/imu', Imu, imucallback)
    
    rawGpsSub=rospy.Subscriber(dronename+'/raw_gps_position',NavSatFix,rawgpscallback)
    gpsPub=rospy.Publisher(dronename+'/GPSmeters',Float64MultiArray, queue_size=1)
    
    packlosspub=rospy.Publisher(dronename+'/PacketLoss',Packet2, queue_size=1)
    packetSub=rospy.Subscriber(dronename+'/Packet',Packet, packetlosscb)
    packetPub2=rospy.Publisher(dronename+'/Packet2',Packet2, queue_size=1)
    
    droneOrientationSub = rospy.Subscriber(dronename+'/attitude',QuaternionStamped, DroneOrientationCB)
    gimbalSub=rospy.Subscriber(dronename+'/gimbal_angle',Vector3Stamped, Gimbal_cb)

    terminalHwPub = rospy.Publisher(dronename+'/TerminalHardware', TerminalHardware, queue_size=1)
    droneHwPub = rospy.Publisher(dronename+'/DroneHardware', DroneHardware, queue_size=1)
    
    batterySub = rospy.Subscriber(dronename+'/battery_state', BatteryState, batteryCB)
    escMotorSub = rospy.Subscriber(dronename+'/ESC_data', EscData, escMotorCB) 
    print('Setting up rostopics FINISHED')       


def publishTerminalHardware():
    hwPckt = GetTerminalHardware()
    terminalHwPub.publish(hwPckt)


def publishDroneHardware():
    global droneHwPub
    
    batteryTempSrv = rospy.ServiceProxy(dronename+'/get_single_battery_dynamic_info', GetSingleBatteryDynamicInfo)
    
    dhwPacket = DroneHardware()
    
    dhwPacket.seq = seq
    dhwPacket.uid = boardId
    
    dhwPacket.batteryThreshold = 10
    dhwPacket.batteryPercentage = int(batteryPercentage)
    dhwPacket.batteryTemperature = -69
    
    '''response = batteryTempSrv(1)
    try:
	if response:
		dhwPacket.batteryTemperature = response.smartBatteryDynamicInfo.batteryTemperature
	else:
		dhwPacket.batteryTemperature = -69
    except:
	print('Battery Temperature Service Error')
	dhwPacket.batteryTemperature = -69'''
    
    
    
    dhwPacket.batteryVoltage = int(batteryVoltage)
    dhwPacket.batteryCurrent = int(batteryCurrent)
    dhwPacket.batteryCapacity = int(batteryCapacity)

    dhwPacket.escCount = escCount
    dhwPacket.escSpeed = escSpeed
    dhwPacket.escVoltage = escVoltage
    dhwPacket.escTemperature = escTemperature
    
    droneHwPub.publish(dhwPacket)    
     

if __name__=='__main__':
    try:   
        print('Initializing Joy Controller Basestation...')
        rospy.init_node('joycontroller_basestation' + '_' + GetBoardID())
        
        rate=rospy.Rate(1)
        precision = 8
        np.set_printoptions(precision = precision)
        
#        platformIp = findPlatformIp()
#        if handshakePlatform():
#            signal.signal(signal.SIGINT, signal_handler)
#        else:
#            print('Could not shake hands with platform...\nExiting...\n')
#            exit(0)
            
        setupTopics()
        
#        print('Setting up Mission Middleware')
#        import telemetryTranslator as  tt
#        import csv2rosMission as mm
#        import orthocamera as oc
#        
#        # Setup Mission Middleware
#        tt.listener(dji_name=dronename)
#        print('telemetryTranslator:\tReady')
#        rate.sleep()
#        mm.listener(dji_name=dronename)
#        print('csv2rosMission:\tReady')
#        
#        oc.listener(platformIp, dji_name=dronename)
#        print('orthocamera:\tReady')       
#        print('Middleware Setup:\tREADY')
#        
#        db.connect()
        
        #import joyController as jc
        #jc.listener(dji_name=dronename)
        
        while not rospy.is_shutdown():
            now = datetime.datetime.now()
            timestmp = now.strftime("%Y%m%d-%H:%M:%S.%s")
            rate.sleep()
            
            #publishTelemetry()
#            publishTerminalHardware()
#            publishDroneHardware()
            
            
            if seq % 100 == 0:
                print(boardId, timestmp)
                
            seq = seq+1
                        
#        db.close()
        
    except rospy.ROSInterruptException: 
        pass
