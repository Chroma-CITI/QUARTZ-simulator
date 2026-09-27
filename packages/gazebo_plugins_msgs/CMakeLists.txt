cmake_minimum_required(VERSION 2.8.3)
project(chroma_gazebo_plugins_msgs)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
)

find_package(catkin REQUIRED message_generation std_msgs geometry_msgs nav_msgs)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  Barometer.msg
  BodyGroundTruth.msg
  Gps.msg
  Uwb.msg
  IRLock.msg
  OpticalFlow.msg
  Rotorspeed.msg
  Wind.msg
  Float.msg
  OdometryArray.msg
)
add_service_files(
 FILES
 UwbStatus.srv
 UwbToken.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs
  DEPENDS EIGEN3
)
