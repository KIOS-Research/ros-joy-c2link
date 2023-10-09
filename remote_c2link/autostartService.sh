#!/bin/bash

# Define a timestamp function
timestamp() {
    date "+%Y/%m/%d-%H:%M:%S"
}

#echo $(ps -aux | grep openvpn)
ps -aux | grep openvpn | awk '{ print $2}' | xargs -n1 sudo kill -9


echo $(timestamp) - Service started

connected_monitors=$(xrandr | grep ' connected' | wc -l)
ip3g=$(lshw 2> /dev/null | grep CDC | grep -oE "\b([0-9]{1,3}\.){3}[0-9]{1,3}\b")
ipTun=$(ip addr show dev tun0 | grep -oE "\b([0-9]{1,3}\.){3}[0-9]{1,3}\b")
echo $(timestamp) - 3G IP: $ip3g

while [ -z "$ip3g" ]
do
    echo $(timestamp) - Polling 3g
    sleep 5
    
    ip3g=$(lshw 2> /dev/null | grep CDC | grep -oE "\b([0-9]{1,3}\.){3}[0-9]{1,3}\b")
    echo $(timestamp) - 3G IP: $ip3g 
done
echo $(timestamp) - 3G OK

machine_id=$(cat /etc/machine-id | cut -c1-4)
echo $(timestamp) - Machine-Id found - $machine_id

echo $(timestamp) - Starting hotspot
./scripts/hotspot.sh &
sleep 5
echo $(timestamp) - Hotspot OK


echo $(timestamp) - Starting VPN connection
sudo openvpn --client --config ./configs/matrice300_$machine_id.ovpn --daemon
sleep 5
echo $(timestamp) - VPN connection Established

while [ -z "$ipTun" ]
do
    echo $(timestamp) - Polling VPN connection
    sleep 5
     
    ipTun=$(ip addr show dev tun0 | grep -oE "\b([0-9]{1,3}\.){3}[0-9]{1,3}\b")
    echo $(timestamp) - TUN IP: $ipTun
done
echo $(timestamp) - VPN OKAY


echo $PWD
directoryOwner=$(echo $PWD | awk -F '/' '{print $3}')
echo "Directory User: $directoryOwner"
#su $directoryOwner
#eval echo ~$USER


echo $(timestamp) - SCRIPT Running...
source devel/setup.bash 
su - -c 'cd Documents/remote_c2link; ./launch/AutostartMultimasterOSDK.sh 2>&1 | tee std.out' $directoryOwner

#echo $(timestamp) - SLEEPING TO INFINITY
#sleep infinity
