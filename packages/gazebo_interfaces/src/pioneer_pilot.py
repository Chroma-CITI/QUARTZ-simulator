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
from math_utils import rotatewithquaternion
from math_utils import quaternioninv

# ROS messages
from nav_msgs.msg import Odometry as Msg_Odometry
from geometry_msgs.msg import Point as Msg_Point
from geometry_msgs.msg import Wrench as Msg_Wrench

# ROS services
from std_srvs.srv import Trigger as Srv_Trigger
from std_srvs.srv import TriggerResponse as Srv_TriggerResponse

# ##CONSTANTS =========================================================
# Default ROS parameters
DEFAULT_FREQ = 10.0

# Constants
G = 9.81


# ##CLASSES ============================================================
class PioneerPilot(object):
    """
    """

    # Initialization function -----------------------------------------
    def __init__(self):
        """
        """
        # Parameters initialization
        self.__init__parameters()
        # State vector
        self.state_ok = False
        self.state_pxyz = None
        self.state_quat = None
        self.state_vxyz = None
        self.state_omgz = None
        # Target
        self.target_pxyz = None
        self.target_list = []
        # Commands history
        self.cmd_tet_prev = 0.0
        self.cmd_phi_prev = 0.0
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
        self.pub_cmd = rospy.Publisher("~cmd", Msg_Wrench, queue_size=10)

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
            if (self.target_pxyz is not None):
                # Distance with the current target
                dist_pxyz = numpy.linalg.norm(self.state_pxyz - self.target_pxyz)
                if (dist_pxyz < 0.5):
                    # Next target
                    target = self.target_list.pop(0)
                    self.target_pxyz = numpy.array([target.x, target.y, target.z])
            else:
                # First target
                target = self.target_list.pop(0)
                self.target_pxyz = numpy.array([target.x, target.y, target.z])

    def compute_cmd(self):
        """
        Command computing
        """
        if (self.target_pxyz is not None):
            # Linear command
            cmd_force_xyz = (self.target_pxyz - self.state_pxyz) - 2.0*self.state_vxyz
            cmd_force_abc = rotatewithquaternion(cmd_force_xyz, quaternioninv(self.state_quat))
            cmd_force_fwd = max(0.0, cmd_force_abc[0])
            # Angular command
            if math.sqrt(cmd_force_xyz[0]**2 + cmd_force_xyz[1]**2) > 0.5:
                cmd_torque_z = math.atan2(cmd_force_abc[1], cmd_force_abc[0]) - 2.0*self.state_omgz
            else:
                cmd_torque_z = 0.0
            # ROS message with commands
            msg = Msg_Wrench()
            msg.force.x = 5.0*math.tanh(cmd_force_fwd)
            msg.force.y = 0.0
            msg.force.z = 5.0*math.tanh(cmd_force_abc[2])
            msg.torque.x = 0.0
            msg.torque.y = 0.0
            msg.torque.z = 2.0*math.tanh(cmd_torque_z)
            # Publish
            self.pub_cmd.publish(msg)

    # Callback functions -----------------------------------------------
    def callback_odometry(self, data):
        """
        """
        # Position in ENU frame
        self.state_pxyz = numpy.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z])
        # Orientation in FLU(t) / ENU frame
        self.state_quat = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w]
        # Linear velocity in ENU frame
        self.state_vxyz = rotatewithquaternion(
            numpy.array([
                data.twist.twist.linear.x,
                data.twist.twist.linear.y,
                data.twist.twist.linear.z]),
            self.state_quat)
        # Angular z velocity in FLU frame
        self.state_omgz = data.twist.twist.angular.z
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
            self.target_pxyz = self.state_pxyz
            self.target_list = []
            ret.success = True
            return ret


# ##MAIN ===============================================================
if __name__ == "__main__":
    rospy.init_node("pilot", anonymous=False)
    pilote = PioneerPilot()
    pilote.run()
