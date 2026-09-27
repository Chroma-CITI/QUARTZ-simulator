#!/bin/bash

if [ $# = "0" ]; then
  echo "Arguments :"
  echo "1 = Chemin absolu du dossier git <PX4-Autopilot> "
  echo "2 = Chemin absolu du dossier git <drone-simulator>"
  exit 0
fi

if [ $1 = "-h" ]; then
  echo "Arguments :"
  echo "1 = Chemin absolu du dossier git <PX4-Autopilot> "
  echo "2 = Chemin absolu du dossier git <drone-simulator>"
  exit 0
fi

if [ $# = "2" ]; then
  x0=0.0
  y0=0.0
  z0=0.0
  # z0=0.0
  psi0_deg=70.0
  psi0_rad=`echo "$psi0_deg * 3.14 / 180.0" | bc -l`
  xterm -e "sudo $1/build/px4_sitl_default/bin/./px4 -s $2/SITL/intelaero.config" &
  xterm -e "roslaunch chroma_gazebo_models drone_world.launch drone:=intelaero_bugwright world:=empty_world x:=$x0 y:=$y0 z:=$z0 yaw:=$psi0_rad; pkill -9 gzclient; pkill -9 gzserver" &
  sleep 10.0
  roslaunch chroma_gazebo_interfaces mavros_intelaero.launch drone:=intelaero_bugwright
  pkill -9 gzclient
  pkill -9 gzserver
else
  echo "Nombre incorrect d'arguments... "
  exit 0
fi
