#!/usr/bin/env python3
# coding: utf-8

# Author : vledoze
# Date   : 25/05/2018

### IMPORTS =======================================================
# Import standard lib
import rospy
import sys

# Import local lib

# ROS messages
from std_msgs.msg import Empty as Msg_Empty
from sensor_msgs.msg import Joy as Msg_Joy
from chroma_gazebo_plugins_msgs.msg import Rotorspeed as Msg_Rotorspeed

# ROS services
from std_srvs.srv import Empty as Srv_Empty

### CONSTANTS =========================================================
DEFAULT_ROTORSPEED_MAX = 68.0
# Motors index
FRONT_RIGHT = 0
BACK_LEFT = 1
FRONT_LEFT = 2
BACK_RIGHT = 3

# Commands constants
PITCH_CONSTANT = 0.1
ROLL_CONSTANT = 0.1
YAW_CONSTANT = 0.1

# Buttons
PS3_BUTTON_ACTION_TRIANGLE   = 0
PS3_BUTTON_ACTION_CIRCLE     = 1
PS3_BUTTON_ACTION_CROSS      = 2
PS3_BUTTON_ACTION_SQUARE     = 3
PS3_BUTTON_REAR_LEFT_1       = 4
PS3_BUTTON_REAR_RIGHT_1      = 5
PS3_BUTTON_REAR_LEFT_2       = 6
PS3_BUTTON_REAR_RIGHT_2      = 7
PS3_BUTTON_START             = 9
PS3_BUTTON_SELECT            = 8
PS3_BUTTON_STICK_LEFT        = 10
PS3_BUTTON_STICK_RIGHT       = 11

# Axes
PS3_AXIS_STICK_LEFT_LEFTWARDS    = 0
PS3_AXIS_STICK_LEFT_UPWARDS      = 1
PS3_AXIS_STICK_RIGHT_LEFTWARDS   = 2
PS3_AXIS_STICK_RIGHT_UPWARDS     = 3
PS3_AXIS_BUTTON_CROSS_LEFTWARD   = 4
PS3_AXIS_BUTTON_CROSS_UPWARD     = 5

### CLASSES ============================================================
class Classe(object):
    """
    """

    # Initialization functions -----------------------------------------
    def __init__(self):
        """
        """
        # Rotation velocities of the 4 motors
        self.values = [0.0, 0.0, 0.0, 0.0]
        self.armed = False
        # Max speed parameter
        self.rotorspeed_max = rospy.get_param("~rotorspeed_max", DEFAULT_ROTORSPEED_MAX)
        # ROS publishers
        self.__init__publishers()
        # ROS subscribers
        self.__init__subscribers()

    def __init__publishers(self):
        """
        ROS publishers initialization
        """
        self.pub_rotorspeed = rospy.Publisher("command/rotorspeed", Msg_Rotorspeed, queue_size=1)

    def __init__subscribers(self):
        """
        ROS subscribers initialization
        """
        rospy.Subscriber("joy", Msg_Joy, self.callback_msg_joy)

    # Public functions ----------------------------------------------
    def run(self):
        """
        """
        rate = rospy.Rate(20) # 20Hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.step()

    def step(self):
        msg_rotorspeed = Msg_Rotorspeed()
        msg_rotorspeed.values = self.values
        self.pub_rotorspeed.publish(msg_rotorspeed)


    # Callback functions -----------------------------------------------
    def callback_msg_joy(self, data):
        """ Callback Joystick """
        #
        cmd_avant_arriere = data.axes[PS3_AXIS_STICK_LEFT_UPWARDS]
        cmd_left_right = data.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]
        cmd_rot = data.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS]
        cmd_up = data.buttons[PS3_BUTTON_REAR_RIGHT_2]
        cmd_down = data.buttons[PS3_BUTTON_REAR_LEFT_2]
        cmd_arm = data.buttons[PS3_BUTTON_ACTION_CROSS]
        cmd_disarm = data.buttons[PS3_BUTTON_ACTION_SQUARE]
        cmd_reset = data.buttons[PS3_BUTTON_ACTION_CIRCLE]
        #Commandes
        s = "Commands : \n"
        #
        if cmd_reset > 0.0:
            self._reset_world()
            return
        if (cmd_arm > 0.0) & (not self.armed):
            s += "  ARMING command \n"
            val = self.rotorspeed_max/100.0
            self.values = [val, val, val, val]
            self.armed = True
            return
        if (cmd_disarm > 0.0) & (self.armed):
            s += "  DISARMING command \n"
            self.values = [0., 0., 0., 0.]
            self.armed = False
            return
        # Thrust command
        if cmd_up > 0.0:
            s += "  UP command \n"
            thrust =  self.rotorspeed_max/100.0
        elif cmd_down > 0.0:
            s += "  DOWN command \n"
            thrust = -self.rotorspeed_max/100.0
        else:
            thrust = 0.0
        # Pitch command
        if cmd_avant_arriere > 0.0:
            s += "  FORWARD command : {0} \n".format(abs(cmd_avant_arriere))
        elif cmd_avant_arriere < 0.0:
            s += "  Backward command : {0} \n".format(abs(cmd_avant_arriere))
        else:
            s += "  NULL FORWARD-BACKWARD command \n"
        #
        if cmd_left_right > 0.0:
            s += "  LEFT command : {0} \n".format(abs(cmd_left_right))
        elif cmd_left_right < 0.0:
            s += "  RIGHT command : {0} \n".format(abs(cmd_left_right))
        else:
            s += "  NULL LEFT-RIGHT command \n"
        #
        if cmd_rot > 0.0:
            s += "  ROT. POS. command : {0} \n".format(abs(cmd_rot))
        elif cmd_rot < 0.0:
            s += "  ROT. NEG. command : {0} \n".format(abs(cmd_rot))
        else:
            s += "  NULL ROT command \n"
        # print self.values
        pitch = cmd_avant_arriere*PITCH_CONSTANT
        roll = cmd_left_right*ROLL_CONSTANT
        yaw = cmd_rot*YAW_CONSTANT
        self.values[FRONT_RIGHT] += thrust - pitch + roll - yaw
        self.values[BACK_LEFT] += thrust + pitch - roll - yaw
        self.values[FRONT_LEFT] += thrust - pitch - roll + yaw
        self.values[BACK_RIGHT] += thrust + pitch + roll + yaw

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
        self.values = [0.0, 0.0, 0.0, 0.0]
        self.armed = False



### MAIN ===============================================================
if __name__=="__main__":
    #Création du noeud
    rospy.init_node("node", anonymous=False)
    node = Classe()
    node.run()
