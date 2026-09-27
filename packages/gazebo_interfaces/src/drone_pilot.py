#!/usr/bin/env python3
# coding: utf-8

# Author : vledoze
# Date   : 01/10/2018
# Author 2: vdufour
# Date    : 29/03/2021

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
from math_utils import D2R
from math_utils import euler2quaternion
from math_utils import quaternion2euler
from math_utils import quaternion2psi
from math_utils import quaternioninv
from math_utils import rotatewithpsi
from math_utils import rotatewithquaternion

# ROS messages
from std_msgs.msg import Empty as Msg_Empty
from sensor_msgs.msg import Imu as Msg_Imu
from nav_msgs.msg import Odometry as Msg_Odometry
from geometry_msgs.msg import Pose as Msg_Pose

# ROS services
from std_srvs.srv import Trigger as Srv_Trigger
from std_srvs.srv import TriggerResponse as Srv_TriggerResponse

# ##CONSTANTS =========================================================
# Default ROS parameters
DEFAULT_FREQ = 10.0
DEFAULT_BORNE_ACC_XY = 0.5
DEFAULT_BORNE_ACC_Z = 0.5
DEFAULT_TARGET_PX = 0.0
DEFAULT_TARGET_PY = 0.0
DEFAULT_TARGET_PZ = 2.0
DEFAULT_TARGET_VX = 0.0
DEFAULT_TARGET_VY = 0.0
DEFAULT_TARGET_VZ = 0.0

# Constants
G = 9.81


# ##CLASSES ============================================================
class DronePilot(object):
    """
    """

    # Initialization functions -----------------------------------------
    def __init__(self):
        """
        """
        # Parameters initialization
        self.__init__parameters()
        # State vector
        self.state_ok = False
        self.state_pxyz = numpy.array([0.0, 0.0, 0.0])
        self.state_quat = numpy.array([0.0, 0.0, 0.0, 1.0])
        self.state_vabc = numpy.array([0.0, 0.0, 0.0])
        # Target
        self.target_pxyz = numpy.array([self.target_px, self.target_py, self.target_pz])
        self.target_vxyz = numpy.array([self.target_vx,self.target_vy, self.target_vz])
        self.target_axyz = numpy.array([0.0, 0.0, 0.0])
        self.target_psi = 0.0
        self.target_dpsi = 0.0
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
        self.borne_acc_xy = rospy.get_param("~borne_acc_xy", DEFAULT_BORNE_ACC_XY)
        self.borne_acc_z = rospy.get_param("~borne_acc_z", DEFAULT_BORNE_ACC_Z)
        self.target_px = rospy.get_param("~target_px", DEFAULT_TARGET_PX)
        self.target_py = rospy.get_param("~target_py", DEFAULT_TARGET_PY)
        self.target_pz = rospy.get_param("~target_pz", DEFAULT_TARGET_PZ)
        self.target_vx = rospy.get_param("~target_vx", DEFAULT_TARGET_VX)
        self.target_vy = rospy.get_param("~target_vy", DEFAULT_TARGET_VY)
        self.target_vz = rospy.get_param("~target_vz", DEFAULT_TARGET_VZ)

    def __init__publishers(self):
        """
        ROS publishers initialization
        """
        self.pub_cmd = rospy.Publisher("~cmd", Msg_Imu, queue_size=10)
        self.pub_home = rospy.Publisher("~back_to_home", Msg_Empty, queue_size=10)

    def __init__subscribers(self):
        """
        ROS subscribers initialization
        """
        rospy.Subscriber("topic_odometry", Msg_Odometry, self.callback_odometry)
        rospy.Subscriber("topic_target", Msg_Pose, self.callback_target)

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

    def define_target_vxyz(self):
        if len(self.target_list)>0:
            next_target = self.target_list[0]
            self.target_vx = numpy.sign(next_target.position.x - self.state_pxyz[0])
            self.target_vy = numpy.sign(next_target.position.y - self.state_pxyz[1])
            self.target_vz = numpy.sign(next_target.position.z - self.state_pxyz[2])
            self.target_vxyz = numpy.array([2*self.target_vx,2*self.target_vy,self.target_vz])
        else:
            self.target_vxyz = numpy.array([self.state_vabc[0],self.state_vabc[1],self.state_vabc[2]])


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
            # Distance with the current target
            dist_pxyz = numpy.linalg.norm(self.state_pxyz - self.target_pxyz)
            # Distance with the desired heading
            dist_psi = 2.0*math.atan(math.tan(0.5*(self.target_psi - quaternion2psi(self.state_quat))))
            if (dist_pxyz < 1.0) & (dist_psi < 5.0*D2R):
                # Next target
                target = self.target_list.pop(0)
                rospy.loginfo("Next target: " + str(target))
                self.target_pxyz = numpy.array([
                    target.position.x,
                    target.position.y,
                    target.position.z])
                self.target_psi = quaternion2psi([
                    target.orientation.x,
                    target.orientation.y,
                    target.orientation.z,
                    target.orientation.w])
                #self.define_target_vxyz()


    def compute_cmd(self):
        """
        Calculation of the command from the search for a point of equilibrium
        between Xt the target position and X the current position

             (Xt - X)'' + 2(Xt - X) + (Xt - X) = 0
         => X'' = Xt'' + 2(Xt - X) + (Xt - X)

        with X'' = R1(psi)*A + R2(psi)*V*w
        with R1 and R2 matrices expressed as a function of the current heading
        with V: the current speed expressed in the drone frame
        with A: the acceleration command expressed in the drone frame
        with w: the speed of rotation around the vertical axis of the drone frame

        Angle and acceleration equivalence calculated with
        m.A = | 0 + T*| cos(phi).sin(tet)
              | 0     | -sin(tet)
              | -m*G  | cos(phi).cos(tet)
        With T the thrust (N)
        and A the accelerations commanded in the Ruvw drone frame
        """
        # Current attitude
        state_psi, state_tet, state_phi = quaternion2euler(self.state_quat)
        # Position errors
        error_xyz = self.target_pxyz - self.state_pxyz
        # Linear velocity errors
        error_vxyz = self.target_vxyz - rotatewithquaternion(self.state_vabc, self.state_quat)
        # Acceleration commands in FLU0 frame
        # Default version
        cmd_axyz = self.target_axyz + 2.0*error_vxyz + error_xyz
        #cmd_axyz = 2.0*error_vxyz
        # Accelerated version
        #cmd_axyz = self.target_axyz + 4.0*error_vxyz + 2.0*error_xyz
        cmd_axyz[0] = max(-self.borne_acc_xy, min(self.borne_acc_xy, cmd_axyz[0]))  # Limits on accelerations ~ maximum tilt
        cmd_axyz[1] = max(-self.borne_acc_xy, min(self.borne_acc_xy, cmd_axyz[1]))  # Limits on accelerations ~ maximum tilt
        cmd_axyz[2] = max(-self.borne_acc_z, min(self.borne_acc_z, cmd_axyz[2]))    # Limits on accelerations ~ safety descend
        cmd_axyz[2] += G                                                            # Gravity compensation
        # Acceleration commands in FLU(t) frame
        cmd_ax1y1z = rotatewithpsi(cmd_axyz, -state_psi)
        # Attitude commands <=> Acceleration commands
        cmd_tet = math.atan2(cmd_ax1y1z[0], cmd_ax1y1z[2])
        cmd_phi = math.atan2(-cmd_ax1y1z[1], cmd_ax1y1z[2]/math.cos(cmd_tet))
        cmd_qat = euler2quaternion(self.target_psi, cmd_tet, cmd_phi)
        # Thrust command
        cmd_thrust = rotatewithquaternion(cmd_axyz, quaternioninv(self.state_quat))[2]
        # Attitude error
        err_psi = 2.0*math.atan(math.tan(0.5*(self.target_psi - state_psi)))
        err_tet = cmd_tet - state_tet
        err_phi = cmd_phi - state_phi
        # Derivative command in attitude
        cmd_dtet = (cmd_tet - self.cmd_tet_prev) * self.freq
        cmd_dphi = (cmd_phi - self.cmd_phi_prev) * self.freq
        self.cmd_tet_prev = cmd_tet
        self.cmd_phi_prev = cmd_phi
        # Rotation transfer inverse matrix
        Ti = numpy.ndarray(
             shape=(3, 3),
             dtype=float,
             buffer=numpy.array(
                 [0.0, -math.sin(state_psi),  math.cos(state_tet)*math.cos(state_psi),
                  0.0,  math.cos(state_psi),  math.cos(state_tet)*math.sin(state_psi),
                  1.0,  0.0,                 -math.sin(state_tet)]))
        # Rotation transfer inverse matrix
        cmd_dA = numpy.ndarray(
             shape=(3, 1),
             dtype=float,
             buffer=numpy.array(
                [self.target_dpsi + err_psi,
                 cmd_dtet + err_tet,
                 cmd_dphi + err_phi]))
        cmd_wabc = rotatewithquaternion(numpy.matmul(Ti, cmd_dA).flatten(), quaternioninv(self.state_quat))
        # ROS message with commands
        msg = Msg_Imu()
        msg.header.stamp = rospy.Time.now()
        msg.orientation.x = cmd_qat[0]
        msg.orientation.y = cmd_qat[1]
        msg.orientation.z = cmd_qat[2]
        msg.orientation.w = cmd_qat[3]
        msg.linear_acceleration.z = cmd_thrust
        msg.angular_velocity.x = cmd_wabc[0]
        msg.angular_velocity.y = cmd_wabc[1]
        msg.angular_velocity.z = cmd_wabc[2]
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
        # Linear velocity in FLU(t) frame
        self.state_vabc = numpy.array([
            data.twist.twist.linear.x,
            data.twist.twist.linear.y,
            data.twist.twist.linear.z])
        # Angular velocity in FLU(t) frame
        self.state_wabc = numpy.array([
            data.twist.twist.angular.x,
            data.twist.twist.angular.y,
            data.twist.twist.angular.z])
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
            self.target_psi = quaternion2psi(self.state_quat)
            self.target_list = []
            ret.success = True
            return ret



# ##MAIN ===============================================================
if __name__ == "__main__":
    rospy.init_node("pilot", anonymous=False)
    pilote = DronePilot()
    pilote.run()
