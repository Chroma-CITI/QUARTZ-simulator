#!/usr/bin/env python3
# coding: utf-8

# Author : vledoze
# Date   : 25/05/2018

### IMPORTS =======================================================
# Import standard lib
import rospy
import sys

# ROS messages
from std_msgs.msg import Empty as Msg_Empty
from chroma_gazebo_plugins_msgs.msg import Rotorspeed as Msg_Rotorspeed

# ROS services
from std_srvs.srv import Empty as Srv_Empty

### CONSTANTS =========================================================
DEFAULT_PARAM = 0.0

### CLASSES ============================================================
class Classe(object):
    """
    """

    # Initialization functions -----------------------------------------
    def __init__(self):
        """
        """
        # Parameters
        self.param = rospy.get_param("~param", DEFAULT_PARAM)
        # ROS subscribers
        self.__init__subscribers()
        # ROS publishers
        self.__init__publishers()

    def __init__publishers(self):
        """
        ROS publishers initialization
        """
        self.pub_rotorspeed = rospy.Publisher("command/rotorspeed", Msg_Rotorspeed, queue_size=1)

    def __init__subscribers(self):
        """
        ROS subscribers initialization
        """
        rospy.Subscriber("topic", Msg_Empty, self.callback_topic)

    # Fonctions publiques ----------------------------------------------
    def run(self):
        """
        """
        rate = rospy.Rate(20) # 20Hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.step()

    def step(self):
        msg_rotorspeed = Msg_Rotorspeed()
        msg_rotorspeed.values = [0.0, 0.0, 0.0, 0.0] #Quadrirotor
        self.pub_rotorspeed.publish(msg_rotorspeed)


    # Callback functions -----------------------------------------------
    def callback_topic(self, data):
        """
        """
        rospy.loginfo("Msg recu")

    # Private funtions -------------------------------------------------
    def _reset_world(self):
        """
        Reset world service
        """
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Srv_Empty)
            reset_world()
            self._reset_attributes()
        except rospy.ServiceException as e:
            print ("Service reset_world call failed: %s" %e)

    def _reset_attributes(self):
        """
        Reset attributes
        """
        return



### MAIN ===============================================================
if __name__=="__main__":
    rospy.init_node("node", anonymous=False)
    node = Classe()
    node.run()
