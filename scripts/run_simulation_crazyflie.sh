#!/bin/bash

xterm -e "roslaunch chroma_gazebo_models drone_world.launch drone:=crazyflie world:=empty_world x:=0.0 y:=0.0 z:=0.0; pkill -9 gzclient; pkill -9 gzserver" &
sleep 10.0
xterm -e "roslaunch chroma_gazebo_interfaces interface_joystick.launch" 
pkill -9 gzclient
pkill -9 gzserver
