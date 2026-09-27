#!/bin/bash

xterm -e "roslaunch chroma_gazebo_models multidrone_world.launch drone:=crazyflie number:=3 world:=empty_world x:=0.0 y:=0.0 z:=0.0; pkill -9 gzclient; pkill -9 gzserver" &
sleep 10.0
xterm -e "roslaunch chroma_gazebo_interfaces multidrone_rl.launch number:=3"
pkill -9 gzclient
pkill -9 gzserver
