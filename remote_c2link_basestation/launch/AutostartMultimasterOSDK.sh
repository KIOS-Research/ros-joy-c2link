#!/bin/bash
### BEGIN INIT INFO
# Provides:          OSDK-ROS
# Short-Description: OSDK-ROS Integration
# Description:       This script starts all required tasks for
#					dji osdk and ros with a multimaster approach
### END INIT INFO

# Author: Christos Georgiades <chr.georgiades [at] gmail.com>

echo "Autostart CWD: $PWD"

SCRIPTPATH="$( cd -- "$(dirname "$0")" </dev/null 2>&1; pwd -P )"
cd $SCRIPTPATH
cd ..
CATKIN_PWD=$(pwd)

echo $SCRIPTPATH
echo $CATKIN_PWD
echo "####"

if [ ! -d "$CATKIN_PWD/devel" ] | [ ! -d "$CATKIN_PWD/build" ]; then
	echo "Missing devel or build directories. Program will exit."
	echo "Run catkin_make before running again"

	exit
fi

#Get Active Ip
ROS_PORT=11311
ROS_IP=$(python $SCRIPTPATH/getActiveIP.py)
ROS_MASTER_URI="http://$ROS_IP:$ROS_PORT"

echo $ROS_IP
echo $ROS_MASTER_URI
echo "####"

netstat -taepn 2>/dev/null | grep 11611
netstat -taepn 2>/dev/null | grep 11611 | cut -d/ -f1 | awk '{print $9}' | xargs kill -9
echo "####"
sleep 7

cust_func(){
	source $CATKIN_PWD/devel/setup.bash
        export "ROS_MASTER_URI=${ROS_MASTER_URI}"
        export "ROS_IP=${ROS_IP}"	

	case "$1" in

   1) 	roscore
	sleep 5
	;;

    2)  rosrun master_discovery_fkie master_discovery
	;;

    3)  rosrun joy joy_node
    	
    	;;

    4)  #python $CATKIN_PWD/src/dji_sdk/scripts/odroidPowerOffServer.py
        roslaunch $SCRIPTPATH/master_sync.launch
	sleep 5
        ;;

    5)  #python $CATKIN_PWD/src/dji_sdk/scripts/telemetryTranslator.py
        python2 $CATKIN_PWD/joyControllerRemotePub.py
	;;

    esac	

}

# For loop 5 times
for i in {1..5}
do
	cust_func $i & # Put a function in the background
	sleep 3
done
 
## Put all cust_func in the background and bash 
## would wait until those are completed 
## before displaying all done message
wait 
