cmake_minimum_required(VERSION 2.8.3)
project(chroma_gazebo_plugins)

add_definitions(-std=c++1y)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  nav_msgs
  gazebo_dev
  gazebo_ros
  chroma_gazebo_plugins_msgs
  cmake_modules
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES

  CATKIN_DEPENDS
  roscpp
  tf
  geometry_msgs
  nav_msgs
  gazebo_dev
  gazebo_ros
  chroma_gazebo_plugins_msgs
)

add_library(gazebo_plugin__body src/gazebo_plugin__body.cpp)
target_link_libraries(gazebo_plugin__body ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__body ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__body_uav src/gazebo_plugin__body_uav.cpp)
target_link_libraries(gazebo_plugin__body_uav gazebo_plugin__body)
add_dependencies(gazebo_plugin__body_uav ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__body_auv src/gazebo_plugin__body_auv.cpp)
target_link_libraries(gazebo_plugin__body_auv gazebo_plugin__body)
add_dependencies(gazebo_plugin__body_auv ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__magnetic_wheel src/gazebo_plugin__magnetic_wheel.cpp)
target_link_libraries(gazebo_plugin__magnetic_wheel ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__magnetic_wheel ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__wind src/gazebo_plugin__wind.cpp)
target_link_libraries(gazebo_plugin__wind ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__wind ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__rotor src/gazebo_plugin__rotor.cpp)
target_link_libraries(gazebo_plugin__rotor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__rotor ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__motor src/gazebo_plugin__motor.cpp)
target_link_libraries(gazebo_plugin__motor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__motor ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__velodyn src/gazebo_plugin__velodyn.cpp)
target_link_libraries(gazebo_plugin__velodyn ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}  ${PCL_LIBRARIES})
add_dependencies(gazebo_plugin__velodyn ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__imu src/gazebo_plugin__imu.cpp)
target_link_libraries(gazebo_plugin__imu ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__imu ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__gps src/gazebo_plugin__gps.cpp)
target_link_libraries(gazebo_plugin__gps ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__gps ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__magnetometer src/gazebo_plugin__magnetometer.cpp)
target_link_libraries(gazebo_plugin__magnetometer ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__magnetometer ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__barometer src/gazebo_plugin__barometer.cpp)
target_link_libraries(gazebo_plugin__barometer ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__barometer ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__laserscan src/gazebo_plugin__laserscan.cpp)
target_link_libraries(gazebo_plugin__laserscan RayPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(gazebo_plugin__laserscan  ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__mavlink_interface src/gazebo_plugin__mavlink_interface.cpp)
target_link_libraries(gazebo_plugin__mavlink_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__mavlink_interface ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__quadcopter_attitude_controller src/gazebo_plugin__quadcopter_attitude_controller.cpp)
target_link_libraries(gazebo_plugin__quadcopter_attitude_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__quadcopter_attitude_controller ${catkin_EXPORTED_TARGETS})

add_library(gazebo_plugin__uwb src/gazebo_plugin__uwb.cpp)
target_link_libraries(gazebo_plugin__uwb ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_plugin__uwb ${catkin_EXPORTED_TARGETS})
