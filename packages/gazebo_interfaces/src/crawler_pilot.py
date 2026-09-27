#!/usr/bin/env python3
# coding: utf-8

# Author : vledoze
# Date   : 01/10/2018

# Coordinates systems :
#   O      : Drone's starting point.
#   M(t)   : Center of the drone at t.
#   ENU    : East-North-Up fixed frame with 0 point as origin.
#            M(t)_ENU = (pe, pn, pu)
#   FLUO   : Front-Left-Up fixed frame with origin M(t=0)=O.
#            M(t)_FLU0 = (px, py, pz)
#   FLU1   : Front-Left-Up fixed frame with origin M(t), with plane (px, py) // p(x1, py1).
#            M(t)_FLU1 = (px1, py1, pz)
#   FLU(t) : Front-Left-Up moving frame with origin M(t).
#            M(t)_FLU(t) = (pa, pb, pc)
#   IJK    : Moving frame describing the measurement axes of any sensor centered on itself.
#            Imu IJK frame != magnetometer IJK frame
#
# Drone's states
#   (px, py, pz)     : M(t) position in FLU0 frame
#   (va, vb, vc)     : M(t) linear velocity in FLU(t) frame
#   (qi, qj, qk, qw) : Quaternion giving the orientation of the FLU(t) frame in the FLU0 frame
#   (wa, wb, wc)     : M(t) angular velovity in the FLU(t) frame
#
# Drone measurement
#   (pe, pn)         : Measurement of the position of the point M(t) projected on the East-North plane of the ENU reference frame
#   (px, py)         : Measurement of the position of the point M(t) projected on the Front-Left plane of the FLU0 reference frame
#   (pu)             : Measurement of the position of the point M(t) projected on the Up axis of the ENU frame
#   (pz)             : Measurement of the position of the point M(t) projected on the Up axis of the FLU0 frame
#   (pc)             : Measurement of the position of the point M(t) projected on the Up axis of the FLU(t) frame
#   (vx, vy, vz)     : Measurement of the speed of the point M(t) in the FLU0 frame
#   (ve, vn)         : Measurement of the speed of the point M(t) projected on the East-North plane of the ENU reference frame
#   (va, vb)         : Measurement of the speed of the point M(t) projected on the Front-Left plane of the reference FLU(t)
#   (qi, qj, qk, qw) : Quaternion expressing the orientation of the FLU(t) frame in the FLU0 frame

# ##IMPORTS =======================================================
# Import standard lib
import rospy
import math
import numpy

# Import local lib
from math_utils import euclidian_norm
from math_utils import quaternion2psi
from math_utils import quaternioninv
from math_utils import quaternionproduct
from math_utils import rotatewithquaternion

# ROS messages
from nav_msgs.msg import Odometry as Msg_Odometry
from geometry_msgs.msg import Point as Msg_Point
from geometry_msgs.msg import Twist as Msg_Twist

# ROS services
from std_srvs.srv import Trigger as Srv_Trigger
from std_srvs.srv import TriggerResponse as Srv_TriggerResponse

# ##CONSTANTS =========================================================
# Default ROS parameters
DEFAULT_FREQ = 10.0


# ##CLASSES ============================================================
class CrawlerPilot(object):
    """
    """

    # Initialization functions  -----------------------------------------
    def __init__(self):
        """
        """
        # Parameters initialization
        self.__init__parameters()
        # Initial pose
        self.p0_enu = None
        self.q_enu2flu0 = None
        # State vector
        self.state_ok = False
        self.state_pxy = None
        self.state_psi = None
        self.state_omg = None
        # Target
        self.target_pxy = None
        self.target_list = []
        # ROS publishers
        self.__init__publishers()
        # ROS subscribers
        self.__init__subscribers()
        # ROS services
        self.__init__services_provided()

    def __init__parameters(self):
        """
        ROS parameters initialization
        """
        self.freq = rospy.get_param("~freq", DEFAULT_FREQ)

    def __init__publishers(self):
        """
        ROS publishers initialization
        """
        self.pub_cmd = rospy.Publisher("~cmd", Msg_Twist, queue_size=10)

    def __init__subscribers(self):
        """
        ROS subscribers initialization
        """
        rospy.Subscriber("topic_odometry", Msg_Odometry, self.callback_odometry)
        rospy.Subscriber("topic_target", Msg_Point, self.callback_target)

    def __init__services_provided(self):
        """
        ROS services initialization
        """
        rospy.Service('~reset_targets', Srv_Trigger, self.service_reset_targets)

    # Public functions ----------------------------------------------
    def run(self):
        """
        """
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            rate.sleep()
            self.step()

    def step(self):
        """
        """
        if self.state_ok:
            # Target choice
            self.update_target()
            # Commands computing
            self.compute_cmd()

    def update_target(self):
        if len(self.target_list) > 0:
            if (self.target_pxy is not None):
                # Distance with the current target
                dist_pxy = euclidian_norm(self.state_pxy - self.target_pxy)
                if (dist_pxy < 0.5):
                    # Next target
                    target = self.target_list.pop(0)
                    self.target_pxy = numpy.array([target.x, target.y])
            else:
                # First target
                target = self.target_list.pop(0)
                self.target_pxy = numpy.array([target.x, target.y])

    def compute_cmd(self):
        """
        Command computing
        """
        if (self.target_pxy is not None):
            # Linear command
            cmd_vit_xy = self.target_pxy - self.state_pxy
            cmd_vit = euclidian_norm(cmd_vit_xy)
            cmd_vit = math.tanh(cmd_vit)
            # Angular command
            if cmd_vit > 0.1:
                cmd_psi = math.atan2(cmd_vit_xy[1], cmd_vit_xy[0])
                cmd_omg = 2.0*math.atan(math.tan((cmd_psi - self.state_psi)/2.0))
            else:
                cmd_omg = -self.state_omg
            # ROS message with commands
            msg = Msg_Twist()
            msg.linear.x = 0.2*cmd_vit
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = cmd_omg
            # Publish
            self.pub_cmd.publish(msg)
        else:
            # ROS message to stay static
            msg = Msg_Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = -self.state_omg
            self.pub_cmd.publish(msg)

    # Callback functions -----------------------------------------------
    def callback_odometry(self, data):
        """
        """
        if not(self.state_ok):
            # Initial pose in ENU frame
            self.p0_enu = numpy.array([
                data.pose.pose.position.x,
                data.pose.pose.position.y,
                data.pose.pose.position.z])
            # Orientation in FLU(t) / ENU frame
            self.q_enu2flu0 = quaternioninv([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w])
        # Position in ENU frame
        state_penu = numpy.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z])
        self.state_pxy = rotatewithquaternion(
            state_penu - self.p0_enu,
            self.q_enu2flu0)[0:2]
        # Orientation in FLU(t) / ENU frame
        state_q_flu2enu = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w]
        self.state_psi = quaternion2psi(quaternionproduct(self.q_enu2flu0, state_q_flu2enu))
        # Angular z velocity in FLU frame
        self.state_omg = data.twist.twist.angular.z
        # OK state
        self.state_ok = True

    def callback_target(self, data):
        """
        """
        self.target_list.append(data)

    def service_reset_targets(self, req):
        """
        """
        ret = Srv_TriggerResponse()
        # NOK state
        if not self.state_ok:
            ret.success = False
            return ret
        # OK state
        else:
            self.target_pxy = self.state_pxy
            self.target_list = []
            ret.success = True
            return ret


# ##MAIN ===============================================================
if __name__ == "__main__":
    rospy.init_node("pilot", anonymous=False)
    pilote = CrawlerPilot()
    pilote.run()
