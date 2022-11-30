# ros-joy-c2link
Remote CnC capabilities using a basestation and remote computer couple

## Table of Contents
- **[C2Link](#C2Link)**<br>
- **[Dependencies](#Dependencies)**<br>
- **[Installation](#Installation)**<br>
- **[Contributions](#Contributions)**<br>

## C2Link
The code was tested in python2.7 with an experimental setup consisting of two Jetson Xavier NX running as a basestation and a client. Connect your controller to the basestation running the code. The setup should transmit the commands from the joystick to the client side computer.

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

For both the basestation and the remote computer you will need to run the following commands.
Make sure you have the following packages installed and up-to-date

```shell
apt install ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep ros-melodic-tf ros-melodic-tf2 ros-melodic-rosserial ros-melodic-rosserial-arduino ros-melodic-joy
```

Then install using pip;

```shell
pip2 install roslibpy
```

## Contributions

[Christos Georgiades](https://github.com/kitos2) - November, 2022

[Nicolas Souli](https://github.com/nsouli02) - November, 2022
