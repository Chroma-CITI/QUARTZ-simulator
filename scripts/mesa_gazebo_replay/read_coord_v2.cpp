#include <cassert>
#include <iostream>
#include <string>
#include <tuple>
#include <iomanip>
#include <list>
#include <fstream>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <algorithm>
#include <typeinfo>
#include <unistd.h>
#include <boost/algorithm/string.hpp>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/forwards.h>
#include <jsoncpp/json/reader.h>

/* 2e version de lecture du json des coordonnées MESA-GAZEBO
Argument unique: nom du fichier json
Fonctionnement: publication dans les topic /new_target point par point sur chacun des topics des drones
Voir read_coord.cpp pour fonctionnement différent.
*/

std::string formatting_message(std::string const x, std::string const y , std::string const z)
{
  int baselong = 16;
  int xl = x.size();
  int yl = y.size();

  std::string message = "\"{position: {x:  , y:  , z:  }, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}\"";
  message.insert(baselong,x);
  message.insert(baselong+6+xl,y);
  message.insert(baselong+12+xl+yl,z);

  return message;

}
///*

void publish_command(Json::Value & logs2,int taille)
{
  int nb_waypts = logs2["0"].size();
  for(int i=0; i<nb_waypts; i++){
    for(int j=0; j<taille;j++){
      std::string k = std::to_string(j);
      std::cout << k << std::endl;
      Json::Value coords = logs2[k][i];

      std::string formatted = formatting_message(coords["x"].toStyledString(),coords["y"].toStyledString(),coords["z"].toStyledString());
      std::ostringstream flux;
      flux << "rostopic pub -1 /intelaero_laserscan_" << k <<"/new_target geometry_msgs/Pose " << formatted;
      std::string publish = flux.str();
      const char *command = publish.c_str();
      std::system(command);
      flux.str("");
      //std::cout << formatted <<std::endl;
      if (i==20){
        flux << "gz world -p 0";
        std::string publish = flux.str();
        const char *command = publish.c_str();
        std::system(command);
        flux.str("");
        std::cout << "Sim Launched" << std::endl;
      }
    }
  }
}

void publish_command(Json::Value & logs2,int taille, char* id)
{
  for(int j=0; j<taille;j++){
    std::cout <<"Drone id: " << *id << std::endl;
    std::cout <<"Step: " << j << std::endl;
    Json::Value coords = logs2[j];
    std::string formatted = formatting_message(coords["x"].toStyledString(),coords["y"].toStyledString(),coords["z"].toStyledString());
    std::ostringstream flux;
    flux << "rostopic pub -1 /intelaero_laserscan_" << *id <<"/new_target geometry_msgs/Pose " << formatted;
    std::string publish = flux.str();
    const char *command = publish.c_str();
    std::system(command);
    flux.str("");
      }
    }

//*/

int main(int argc, char** argv)
{

  Json::Value logs;
  std::string filename = argv[1];
  std::ifstream jfile(filename);
  jfile >> logs;
  int taille = logs.size();
  if (argc > 1){
    publish_command(logs,taille,argv[2]);
  }
  else{
    publish_command(logs,taille);
  }

  return 0;
}
