#!/usr/bin/env python3
# coding: utf-8

# Author : vledoze
# Date   : 25/05/2018

### IMPORTS =======================================================
# Import standard lib
import math
import numpy
import random
import rospy
import sys
import time

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

import matplotlib
import matplotlib.pyplot as plt

from collections import namedtuple
from torch.autograd import Variable

# ROS messages
from std_msgs.msg import Empty as Msg_Empty
from std_msgs.msg import Float32 as Msg_Float32
from sensor_msgs.msg import Imu as Msg_Imu
from chroma_gazebo_plugins_msgs.msg import Rotorspeed as Msg_Rotorspeed
from chroma_gazebo_plugins_msgs.msg import BodyGroundTruth as Msg_BodyGroundTruth

# ROS services
from std_srvs.srv import Empty as Srv_Empty

### CONSTANTS =========================================================
# One episode limit time
EPISODE_STEPS_LIMIT = 100

### CLASSES ============================================================
class MultiDroneReinforcementLearning(object):
    """
    """

    # Initialization functions -----------------------------------------
    def __init__(self):
        """
        """
        # Parameters
        self.__init__parameters()
        # Episodes attributes
        self.__init__episodes_attributes()
        # ROS subscribers
        self.__init__subscribers()

    def __init__parameters(self):
        """
        ROS parameters initialization
        """
        #,Drone type
        self.drone = rospy.get_param("~drone", "crazyflie")
        # Number of drones
        self.drone_number = rospy.get_param("~drone_number", 0)

    def __init__episodes_attributes(self):
        """
        Attributes that will be reset with each new episode
        """
        # Time step per episode
        self.episode_steps = 0
        self.episode_steps_limit = \
            rospy.get_param("~episode_steps_limit", EPISODE_STEPS_LIMIT)*random.random()

    def __init__subscribers(self):
        """
        ROS subscribers initialization
        """
        self.stop_pub_list = []
        self.start_pub_list = []
        # Subscription
        for i in range(self.drone_number):
            self.stop_pub_list.append(rospy.Publisher('/{0}_{1}/stop_learning'.format(self.drone, i+1), Msg_Empty, queue_size=1))
            self.start_pub_list.append(rospy.Publisher('/{0}_{1}/start_learning'.format(self.drone, i+1), Msg_Empty, queue_size=1))


    # Public functions ----------------------------------------------
    def run(self):
        """
        """
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.step()

    def step(self):
        """
        Step
        """
        # Step increment
        self.episode_steps += 1
        if self.episode_steps > self.episode_steps_limit:
            self._reset_world()


    # Private functions -------------------------------------------------
    def _reset_world(self):
        """
        ROS services initialization
        """
        # Stop drones learning
        for stop_pub in self.stop_pub_list:
            stop_pub.publish(Msg_Empty())
        # Gazebo reset
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Srv_Empty)
            reset_world()
        except rospy.ServiceException as e:
            print ("Service reset_world call failed: %s" %e)
        # Resetting episode variables
        self.__init__episodes_attributes()
        # Restarting drone learning
        for start_pub in self.start_pub_list:
            start_pub.publish(Msg_Empty())



### MAIN ===============================================================
if __name__=="__main__":
    rospy.init_node("node", anonymous=False)
    node = MultiDroneReinforcementLearning()
    plt.figure()
    node.run()
