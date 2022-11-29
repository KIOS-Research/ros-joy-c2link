# ros-database-synchronization
Multi-system database logging and synchronization for ROS

## Table of Contents

- **[Introduction](#Dependencies)**<br>
- **[Installation](#Installation)**<br>
- **[Logging](#Logging)**<br>
- **[Synchronization](#Synchronization)**<br>
- **[Contributions](#Contributions)**<br>

## Dependencies
ros-melodic-desktop-full
python-rosdep 
python-rosinstall 
python-rosinstall-generator 
python-wstool 
build-essential 
python-rosdep
ros-melodic-tf ros-melodic-tf2
ros-melodic-rosserial 
ros-melodic-rosserial-arduino
ros-melodic-joy

roslibpy

## Installation
```shell
apt install ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep ros-melodic-tf ros-melodic-tf2 ros-melodic-rosserial ros-melodic-rosserial-arduino ros-melodic-joy```


```shell
pip2 install roslibpy```

## C2Link
The code was tested in python2.7 with an experimental setup with two Jetson Xavier NX running as a basestation and a client. Connect your controller to the basestation running the code. The setup should transmit the commands from the joystick to the client side computer.

## Contributions

[Christos Georgiades](https://github.com/kitos2) - November, 2022
[Nicolas Souli](nsouli02@ucy.ac.cy) - November, 2022
