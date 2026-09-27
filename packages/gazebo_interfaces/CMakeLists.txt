cmake_minimum_required(VERSION 2.8.3)
project(chroma_gazebo_interfaces)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    chroma_gazebo_plugins_msgs
)

catkin_python_setup()

catkin_package()
