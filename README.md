# Vacuum Cleaner Like Robot Software Implementation Example for Dagozilla ITB Internship 2022

## Author
- Farrel Ahmad (Dagozilla ITB Programming Crew)
- Agustinus Yudhistira W. S. (Dagozilla ITB Electrical Crew)

<br>

## Introduction
This program is a software impelmentation example for vaccum cleaner like robot that can avoid any obstacle using three ultrasonic sensors placed in the robot. The sensors are placed at the front, left, and right of the robot. The algorithm is simple, the robot will avoid any obstacle whenever the sensor reading exceed certain distance threshold. Although it is named vacuum cleaner robot. The robot does not have vacuum cleaner in it, only the algorithm, movement, and implementation are similar to the real vacuum cleaner robot.

<br>

## Robot used
1. Implemented by Group 1 Dagozilla Interns 2022
![](https://i.ibb.co/0sVKHjz/LINE-ALBUM-Pertandingan-220814-26.jpg)

<br>

2. Implemented by Group 2 Dagozilla Interns 2022
![](https://i.ibb.co/WvwHyHg/LINE-ALBUM-Pertandingan-220814-20.jpg)

<br>

## Requirements
1. Ubuntu 20.04 LTS
2. ROS Noetic
3. Arduino IDE

<br>

## Installation
1. For ROS in Robot's PC
```sh
$ cd ros-pc/vac_ws
$ catkin_make
```

2. For ROS in Arduino, open `hardware.ino` in ros-arduino in Arduino IDE and upload the code to the desired Arduino. Don't forget to set the pin before uploading. The default baud rate is set to 57600 and can be changed in `rosserial.launch` in ros-pc and in Arduino IDE.

<br>

## How to run
1. Make sure the robot's PC is in the same network as the laptop
2. Plug the Arduino to the PC using USB.
3. Remotly ssh to the robot's PC by using this command
```sh
$ ssh -XC <pc_name>@<ip_address>
# for example
$ ssh -XC laplace@192.168.1.101
```
4. Enter the PC password and now the laptop has connected to PC's terminal
5. Change directory to ros-pc/vac_ws and run this command
```sh
$ source devel/setup.bash #if using bash terminal
# or
$ source devel/setup.zsh #if using zsh terminal
```
6. Launch the program
```sh
$ roslaunch vac_launch routine.launch
```
7. Call rosservice to enable send non zero pwm to arduino
```sh
$ rosservice call /control/base/set_base_active "data : true"
```
8. The robot will now move and avoid any obstacle found by ultrasonic sensor