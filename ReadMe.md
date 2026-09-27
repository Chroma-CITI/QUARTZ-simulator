# Drone Simulator


This project is a Gazebo+ROS based simulator that aims to make Unmaned Aerial Vehicle simulations easier to use and customize. The project’s philosophy is to create gazebo plugins and definition files for mechanical parts, sensors, and interfaces as generic as possible in order to use later them as plug and play “lego” bricks. The components are developed to be as physical representative as possible, especially on the aerodynamics of the UAV’s mechanical parts. The project allows also “Software-In-The-Loop” simulation with the PX4 firmware for flight controllers (see https://github.com/PX4/PX4-Autopilot).
For further information, a [Wiki](https://gitlab.inria.fr/chroma/drones/drone-simulator/-/wikis/home) is available for this project.

![Alt text](media/Capture d'écran 2018-10-23 09:51:13.png?raw=true "Simulated IntelAero & Crazyflie")

![Alt text](media/clip_drone_simulator.gif?raw=true "Gif Simulated IntelAero & Crazyflie")

The simulator includes the following functionality:
* Simulation of the mechanical behavior of an Unmanned Aerial Vehicle
  * Modeling of the body's aerodynamics with lift, drag and moment
  * Modeling of rotors' aerodynamics using the forces and moments' expressions from Philppe Martin's and Erwan Salaün's 2010 IEEE Conference on Robotics and Automation paper "The True Role of Accelerometer Feedback in Quadrotor Control"
* Gives groundtruth informations
  * Positions in East-North-Up reference frame
  * Linear velocity in East-North-Up and Front-Left-Up reference frames
  * Linear acceleration in East-North-Up and Front-Left-Up reference frames
  * Orientation from East-North-Up reference frame to Front-Left-Up reference frame (Quaternions)
  * Angular velocity of Front-Left-Up reference frame expressed in Front-Left-Up reference frame
* Simulation of the following sensors
  * Inertial Measurement Unit with 9DoF (Accelerometer + Gyroscope + Orientation)
  * Barometer using an ISA model for the troposphere (valid up to 11km above Mean Sea Level)
  * Magnetometer with the earth magnetic field declination
  * GPS Antenna with a geodesic map projection
  * UWB distance measurement

This project took pieces of code from the following repositories :

* [PX4 firmware](https://github.com/PX4/PX4-Autopilot)
* [SITL gazebo](https://github.com/PX4/PX4-SITL_gazebo-classic)
* [RotorS simulator](https://github.com/wilselby/rotors_simulator)
* [ROS Quadrotors simulator](https://github.com/wilselby/ROS_quadrotor_simulator)

In this guide, the following variable will be used:

| Variable              | Description                                            | Suggested value                                     |
|-----------------------|--------------------------------------------------------|-----------------------------------------------------|
| `DIR_DRONE_SIMULATOR` | Cloned directory of this repository                    | `${HOME}/drone-simulator`                           |
| `DIR_DRONE_MODELS`    | Directory which contains drone models definitions files| `$DIR_DRONE_SIMULATOR/packages/gazebo_models/models`|
| `DIR_WORLDS`          | Directory which contains worlds definitions files      | `$DIR_DRONE_SIMULATOR/packages/gazebo_worlds/worlds`|
| `DIR_PX4_FIRMWARE`    | Clone directory of PX4 firmware repository             | `${HOME}/PX4-Autopilot`                                       |

## Installation

You need to install at least [ROS Kinetic & Gazebo 7](http://wiki.ros.org/kinetics/Installation/Ubuntu) but it has also been tested with [ROS Melodic & Gazebo 9](http://wiki.ros.org/melodic/Installation/Ubuntu) and [ROS Noetic & Gazebo 11](http://wiki.ros.org/noetic/Installation/Ubuntu).

Install
~~~sh
git clone https://gitlab.inria.fr/chroma/drone-simulator.git
cd drone-simulator
./init.sh
export DIR_DRONE_SIMULATOR=`pwd`
~~~

Create ROS workspace (if not already created)
~~~sh
mkdir -p ~/drone-simulator_ws/src
cd ~/drone-simulator_ws/src
catkin_init_workspace
ln -s $DIR_DRONE_SIMULATOR/packages .
cd ~/drone-simulator_ws
ln -s $DIR_DRONE_SIMULATOR/scripts .
chmod +x scripts/*.sh
catkin build #use catkin_make instead on old ros version
~~~

## Launching the simulator

Supported drone types in `DIR_DRONE_MODELS:=$DIR_DRONE_SIMULATOR/packages/gazebo_models/models`

| Name                  | Description                                           | Description file                                    |
| --------------------- | ----------------------------------------------------- | --------------------------------------------------- |  
| *auv*                 | Autonomous Underwater Vehicle                         | `$DIR_DRONE_MODELS/auv/model.xacro`                 |                   
| *crawler*             | Two wheels ground vehicle                             | `$DIR_DRONE_MODELS/crawler/model.xacro`             |                   
| *crazyflie*           | Crazyflie drone                                       | `$DIR_DRONE_MODELS/crazyflie/model.xacro`           |                   
| *intelaero*           | Intelaero drone                                       | `$DIR_DRONE_MODELS/intelaero/model.xacro`           |
| *intelaero_bugwright* | Intelaero drone model for BugWright2 simulations      | `$DIR_DRONE_MODELS/intelaero_bugwright/model.xacro` |
| *intelaero_chroma*    | Intelaero drone as used by Chroma team                | `DIR_DRONE_MODELS/intelaero_chroma/model.xacro`     |
| *intelaero_laserscan* | Intelaero drone with embedded velodyne for 3D mapping | `DIR_DRONE_MODELS/intelaero_laserscan/model.xacro`  |
| *uav_coaxial*         | Coaxial drone                                         | `DIR_DRONE_MODELS/uav_coaxial/model.xacro`          |
| *px4_vision*          | PX4 Vision drone                                      | `DIR_DRONE_MODELS/px4_vision/model.xacro`           |

Supported worlds in `DIR_WORLDS:=$DIR_DRONE_SIMULATOR/packages/gazebo_worlds/worlds`

| Name        | Description | Description file                |
| ----------- | ----------- | ------------------------------- |  
| boat        | boat        | `$DIR_WORLDS/boat.world`        |
| calibration | calibration | `$DIR_WORLDS/calibration.world` |
| city        | city        | `$DIR_WORLDS/city.world`        |
| empty_world | empty world | `$DIR_WORLDS/empty_world.world` |
| kitchen     | kitchen     | `$DIR_WORLDS/kitchen.world`     |
| outdoor     | outdoor     | `$DIR_WORLDS/boat.world`        |

To launch a **drone simulation**
~~~sh
cd ~/drone-simulator_ws
source devel/setup.bash
roslaunch chroma_gazebo_models drone_world.launch drone:=<drone name> world:=<world name>
~~~

The starting coordinates and attitude of the drone can be set with the arguments
`x:=value`, `y:=value`, `z:=value`, `roll:=value`, `pitch:=value`
and `yaw:=value`. These coordinates are in a East-North-Up reference frame.

In the beginning, the simulation is paused. To start it you have to click on the
play button at the bottom of the gazebo window. Otherwise you can also launch
the simulator with the argument `paused:=False`.

To launch a **multiple drone simulation**
~~~sh
cd ~/drone-simulator_ws
source devel/setup.bash
roslaunch chroma_gazebo_models multidrone_world.launch drone:=<drone name> world:=<world name>  number:=<number of drones to spawn>
~~~

## Using the simulator

The gazebo plugins which simulate the differents parts of the drone
communicate with ROS topics. So to give orders to the drone or to read
its sensors you must run a ROS node. Some examples are given in the folder
*"~/drone-simulator_ws/src/chroma_gazebo_interfaces/src/"*. Further informations are avaiables
on the ROS wiki page http://wiki.ros.org/ROS/Tutorials .

It's easier to use the given launch files (see *"~/drone-simulator_ws/src/chroma_gazebo_interfaces/launch/"*) to run a node.
For example :
~~~sh
roslaunch chroma_gazebo_interfaces joystick.launch
~~~

will give you the control of a simulated crazyflie with an PS4 controller.

Severals shell scripts are also avaiables in the folder *"~/drone-simulator_ws/"*.
For example *"run_simulation_multicrazyflie_rl.sh"* will create 3 simulated
crazyflie drones and will run a reinforcement learning node to make them
learn how to fly and maintain their altitude at 1m above the ground.

## SITL Simulator (Software In The Loop)

TODO : gyro, compass calibration issue and link the px4 controller to the body frame of the Intelaero.

How to use PX4 flight controller software to pilot the simulated drone.
Tested on Ubuntu 20.04 Noetic, and PX4-Autopilot v1.14.0.

### IntelAero

First you might need to install these dependencies
~~~sh
sudo apt update
sudo apt install gcc-arm-none-eabi genromfs python-jinja2
pip install --user numpy toml
~~~

For the Intelaero, we need to download the PX4 firmware from Github :
~~~sh
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.14.0
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl gazebo
export DIR_PX4_FIRMWARE=`pwd`
~~~

To launch everything at once (firmware + Gazebo simulation)
~~~sh
cd ~/drone-simulator_ws
./scripts/run_simulation_intelaero.sh $DIR_PX4_FIRMWARE $DIR_DRONE_SIMULATOR
~~~
