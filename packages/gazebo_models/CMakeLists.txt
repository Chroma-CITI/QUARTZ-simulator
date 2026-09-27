cmake_minimum_required(VERSION 2.8.3)
project(chroma_gazebo_models)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
)

catkin_package(

  CATKIN_DEPENDS
  gazebo_ros

)
