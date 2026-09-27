#!/bin/bash

#Argument nom du fichier contenant les logs
log_file="$1" #Nom du fichier contenant les logs
splitted="$2" #Si "True" créer un terminal par drone pour publier dans les rostopics en //

#Récupère le nombre de drone à appeler dans la simulation
nb=` jq '. | length' $log_file`

#On s'assure que gazebo ne tourne pas
pkill -9 gzclient
pkill -9 gzserver

#On ouvre un monde
xterm -e roslaunch chroma_gazebo_interfaces multi_pilot_traj.launch world:=boat &
sleep 5.0

i=0
#On fait apparaitre le nombre de drone voulus (avec chacun leur contrôleur)
while [ $i -lt $nb ]
do 
  pos_x=` jq '."'$i'"[0]["x"]' $log_file`
  pos_y=` jq '."'$i'"[0]["y"]' $log_file`
  pos_z=` jq '."'$i'"[0]["z"]' $log_file`
  #ATTENTION CHANGEMENT DE REFERENTIEL EFFECTUE LORS DU PASSAGE DE COORDS
  pos_x_gazebo=$pos_y
  pos_y_gazebo=`expr $((-$pos_x))`  
  xterm -hold -e "roslaunch chroma_gazebo_interfaces drone_pilot.launch \
		  x:=$pos_x_gazebo y:=$pos_y_gazebo z:=$pos_z \
		  target_px:=$pos_x target_py:=$pos_y target_pz:=$pos_z \
		  drone_name:=intelaero_laserscan_'$i'" &
  i=$(($i+1))
  sleep 2.0

done
sleep 5.0

#On lance le script de publication des coordonnées.
if [ $splitted = "true" ]
then
  j=0
  while [ $j -lt $nb ] 
  do
    gnome-terminal -- bash -c "./readjson intelaero_laserscan_'$j'.json $j ; bash" &
  j=$(($j+1))
  sleep 1.0
  done
else
  gnome-terminal -- bash -c "./readjson $log_file ; bash"
fi

echo "Done"


#On force tous les noeuds à mourir
#rosnode kill -a
#On force Gazebo à mourir avant de quitter
#pkill -9 gzclient
#pkill -9 gzserver
