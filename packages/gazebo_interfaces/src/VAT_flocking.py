#!/usr/bin/env python3
# coding: utf-8

# Author : jofaure
# Date   : 25/04/2022


# ##IMPORTS =======================================================
# Import standard lib
import rospy
import math
import numpy as np
import os, time
import tf

# Import local lib
from math_utils import D2R
from math_utils import euler2quaternion
from math_utils import quaternion2euler
from math_utils import quaternion2psi
from math_utils import quaternioninv
from math_utils import rotatewithpsi
from math_utils import rotatewithquaternion
from math_utils import SigmoidLin
from math_utils import SigmoidLike
from math_utils import VelDecayLinSqrt
from math_utils import psi2quaternion
from math_utils import perpendicular


# ROS messages
from std_msgs.msg import Empty as Msg_Empty
from sensor_msgs.msg import Imu as Msg_Imu
from nav_msgs.msg import Odometry as Msg_Odometry
from geometry_msgs.msg import Pose as Msg_Pose
from rosgraph_msgs.msg import Clock as Msg_Clock
from chroma_gazebo_plugins_msgs.msg import Uwb as Msg_Uwb
from sensor_msgs.msg import LaserScan as Msg_LaserScan
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform

# ROS services
from std_srvs.srv import Trigger as Srv_Trigger
from std_srvs.srv import TriggerResponse as Srv_TriggerResponse

# ##CONSTANTS =========================================================
# Default ROS parameters
DEFAULT_FREQ = 5.0
DEFAULT_BORNE_ACC_XY = 5
DEFAULT_BORNE_ACC_Z = 5
DEFAULT_TARGET_PX = 0.0
DEFAULT_TARGET_PY = 0.0
DEFAULT_TARGET_PZ = 2.0
#DEFAULT_STATE_VX = 2.0
#DEFAULT_STATE_VY = 2.0
DEFAULT_STATE_VZ = 0.0
DEFAULT_STATE_VX = 0*np.random.rand(1)[0]
DEFAULT_STATE_VY = 0*np.random.rand(1)[0]
DEFAULT_TARGET_VX = 0.0
DEFAULT_TARGET_VY = 0.0
DEFAULT_TARGET_VZ = 0.0
DEFAULT_NUM_NEIGHBORS = 4
DEFAULT_DETECTOR_RANGE = 2
DEFAULT_NMAX = 4
DEFAULT_WEIGHT = 1.03

# Constants
G = 9.81



# ##CLASSES ============================================================
class DroneFlockingPilot(object):

    # Initialization functions -----------------------------------------
    def __init__(self):
        """
        """
        # Parameters initialization
        self.__init__parameters()
        # State vector
        self.state_ok = False
        self.state_pxyz = np.array([self.state_px,self.state_py, self.state_pz])
        self.state_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.state_vabc = np.array([self.state_vx,self.state_vy, 0.0])
        self.state_accel = np.array([0.0,0.0,0.0])
        self.desired_speed = np.array([0.0,0.0,0.0])
        self.total_accel =[np.array([0.0,0.0,0.0])]
        self.target_psi = 0
        self.target_dpsi = 0.0
        #self.max_speed = 20
        self.max_speed = 7
        self.msgs_uwb = []
        self.laser_ranges = []
        self.inf_count = 0
        self.count = 0
        self.receivd_pwd = None
        self.power_list = []
        #self.nmax = 4 # Number of neighbors chosen to compute command
        self.obstacle = []
        self.laser_time = 0.0
        # Target
        self.target_pxyz = np.array([self.target_px, self.target_py, self.target_pz])
        self.target_list = []
        self.target_vxyz = np.array([self.target_vx,self.target_vy, 0.0])
        self.target_axyz = np.array([0.0, 0.0, 0.0])
        self.friend_list = []
        self.R_trg = 0.5
        self.d_trg = 1
        self.last_max = np.array([0.0, 0.0, 0.0])
        self.sign_dir = np.array([0.0, 0.0, 0.0])
        # Commands history
        self.cmd_psi_prev = 0.0
        self.cmd_tet_prev = 0.0
        self.cmd_phi_prev = 0.0
        # State vector from a nearby drone (friend)
        self.friend_state_pxyz = self.target_pxyz
        #self.friend_state_twist = np.array([np.random.rand(1)[0],np.random.rand(1)[0], 0.0])
        self.friend_state_twist = np.array([self.state_vx,self.state_vy, 0.0])
        self.friend_state_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.friend_essid = None
        self.essid = None
        self.distance = None # Distance from the neighbor
        self.dist_pxyz = 0
        self.dist_psi = 0
        self.first_round = True
        self.detect_dist = 1.9
        # Flocking parameters for VAT (flockingparam.dat file)
        self.V_flock_l = 4
        self.V_Rep_l = 6.197
        self.V_Frict_l = 0.63
        self.V_Shill_l = 13.622
        self.R_0_l = 5
        self.R_0_Frict = 85.3
        self.R_0_Shill = 0.3
        self.p_l_repuls = 0.13
        self.p_l_attract = 0.006
        self.p_l_frict = 3.2
        self.p_l_shill = 3.55
        self.C_Frict_l = 0.05
        self.a_l_shill = 53
        self.a_l_frict = 4.16

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
        self.state_px = rospy.get_param("~state_px", DEFAULT_TARGET_PX)
        self.state_py = rospy.get_param("~state_py", DEFAULT_TARGET_PY)
        self.state_pz = rospy.get_param("~state_pz", DEFAULT_TARGET_PZ)
        self.state_vx = rospy.get_param("~state_vx", DEFAULT_STATE_VX)
        self.state_vy = rospy.get_param("~state_vy", DEFAULT_STATE_VY)
        self.state_vz = rospy.get_param("~state_vz", DEFAULT_STATE_VZ)
        self.target_px = rospy.get_param("~target_px", DEFAULT_TARGET_PX)
        self.target_py = rospy.get_param("~target_py", DEFAULT_TARGET_PY)
        self.target_pz = rospy.get_param("~target_pz", DEFAULT_TARGET_PZ)
        self.target_vx = rospy.get_param("~target_vx", DEFAULT_TARGET_VX)
        self.target_vy = rospy.get_param("~target_vy", DEFAULT_TARGET_VY)
        self.target_vz = rospy.get_param("~target_vz", DEFAULT_TARGET_VZ)
        self.num_neighbors = rospy.get_param("~num_neighbors",DEFAULT_NUM_NEIGHBORS) #Number of total neighbors (!= nmax)
        self.detector_range = rospy.get_param("~detector_range",DEFAULT_DETECTOR_RANGE) #Number of total neighbors (!= nmax)
        self.nmax = rospy.get_param("~nmax",DEFAULT_NMAX)


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
        rospy.Subscriber("uwb", Msg_Uwb, self.callback_uwb, queue_size=100)
        rospy.Subscriber("imu",Msg_Imu,self.callback_imu)
        rospy.Subscriber("object_avoidance/laser_scan",Msg_LaserScan,self.callback_oa)


    def __init__services_provided(self):
        """
        ROS services initialization
        """
        rospy.Service('~reset_targets', Srv_Trigger, self.service_reset_targets)



    # Public functions ----------------------------------------------
    def align(self):
        """
        VAT alignment force
        """
        neighbor_vel = rotatewithquaternion(self.friend_state_twist, self.friend_state_quat)
        VelDiff = (neighbor_vel - rotatewithquaternion(self.state_vabc, self.state_quat))
        NormVelDiff = np.linalg.norm(VelDiff)
        VelDiff /= NormVelDiff
        MaxVelDiff = max(self.V_Frict_l,VelDecayLinSqrt(self.distance,self.p_l_frict,self.a_l_frict,NormVelDiff,self.R_0_Frict))
        if(NormVelDiff > MaxVelDiff):
            VelDiff *= self.C_Frict_l*(NormVelDiff-MaxVelDiff)
            #print("Align state: {}, {}".format(self.essid,VelDiff))
            return VelDiff*self.freq
        else:
            #print("Alignment state: {}, {}".format(self.essid,np.zeros(3)))
            return np.zeros(3)


    def attraction(self):
        """
         VAT attraction force
        """
        if self.distance > (self.R_0_l): # Or R_0_l + 0.5
            PosDiff = self.state_pxyz - self.friend_state_pxyz
            PosDiff /= np.linalg.norm(PosDiff)
            PosDiff*=SigmoidLin(self.distance,self.p_l_attract,self.V_Rep_l,self.R_0_l) # Or R_0_l + 0.5
            #print("Attraction state: {}, {}".format(self.essid,PosDiff))
            return PosDiff*self.freq
        else:
            #print("Attraction state: {}, {}".format(self.essid,np.zeros(3)))
            return np.zeros(3)


    def repulsion(self):
        """
        VAT repulsion force
        """
        if self.distance <= self.R_0_l:
            PosDiff = self.state_pxyz - self.friend_state_pxyz
            PosDiff /= np.linalg.norm(PosDiff)
            PosDiff*=SigmoidLin(self.distance,self.p_l_repuls,self.V_Rep_l,self.R_0_l)
            #print("Repulsion state: {}, {}".format(self.essid,PosDiff))
            return PosDiff*self.freq
        else:
            #print("Repulsion state: {}, {}".format(self.essid,np.zeros(3)))
            return np.zeros(3)


    def shill(self):
        """
        VAT obstacle avoidance
        """
        if (len(self.obstacle)>1 and min(self.obstacle.keys())<self.detect_dist and self.laser_time>5.0):
            dist_to_obstacle = min(self.obstacle.keys())
            # Laser covers 59 degres
            # alpha = self.obstacle[dist_to_obstacle] - 29.49
            # Laser covers 180 degres
            alpha = self.obstacle[dist_to_obstacle] - 90
            alpha = (np.pi*alpha/180)
            if (self.inf_count<18):  # Obstacle
                x_obs = dist_to_obstacle * np.cos(alpha)
                y_obs = dist_to_obstacle * np.sin(alpha)
                vect_obst = np.array([x_obs - self.state_px, y_obs - self.state_py])
                vect_obst /= np.linalg.norm(vect_obst)
                vect_perp = perpendicular(vect_obst)
                # Multiplicate V_Shill_l by the perpendicular vector of the nearest position of the obstacle vector
                #VelShill = (rotatewithquaternion(self.state_vabc, self.state_quat) - self.V_Shill_l * np.array([-np.sin(alpha), np.cos(alpha), 0.0]))
                if alpha<0:
                    #VelShill = (rotatewithquaternion(self.state_vabc, self.state_quat) - self.V_Shill_l * np.array([np.sin(alpha), -np.cos(alpha), 0.0]))
                    VelShill = (rotatewithquaternion(self.state_vabc, self.state_quat) + rotatewithquaternion(self.V_Shill_l * np.array([vect_perp[0], vect_perp[1], 0.0]), self.state_quat))
                else:
                    #VelShill = (rotatewithquaternion(self.state_vabc, self.state_quat) - self.V_Shill_l * np.array([-np.sin(alpha), np.cos(alpha), 0.0]))
                    VelShill = (rotatewithquaternion(self.state_vabc, self.state_quat) - rotatewithquaternion(self.V_Shill_l * np.array([vect_perp[0], vect_perp[1], 0.0]), self.state_quat))
                NormVelShill = np.linalg.norm(VelShill)
                VelShill /= NormVelShill
                MaxVelShill = VelDecayLinSqrt(dist_to_obstacle,self.p_l_shill,self.a_l_shill,NormVelShill,self.R_0_Shill)
                if (NormVelShill > MaxVelShill):
                    VelShill *= (NormVelShill-MaxVelShill)
                    return VelShill*self.freq
                else:
                    return np.zeros(3)
            elif (self.inf_count>=18):  # Wall
                obst_coord = np.array([dist_to_obstacle * np.sin(alpha), dist_to_obstacle * np.cos(alpha), self.state_pz])  #coord en z peuvent s'obtenir selon étage où la dist est min.
                if dist_to_obstacle<0:
                    PosDiff = obst_coord - self.state_pxyz
                else:
                    PosDiff = self.state_pxyz - obst_coord
                PosDiff /= np.linalg.norm(PosDiff)
                PosDiff *= self.V_Shill_l
                VelDiff = rotatewithquaternion(self.state_vabc, self.state_quat) - PosDiff
                NormVelDiff = np.linalg.norm(VelDiff)
                VelDiff /= NormVelDiff
                MaxVelDiff = VelDecayLinSqrt(dist_to_obstacle,self.p_l_shill,self.a_l_shill,NormVelDiff,self.R_0_Shill)
                if (NormVelDiff > MaxVelDiff):
                    VelDiff *= (NormVelDiff - MaxVelDiff)
                    return VelDiff*self.freq
        else:
            return np.zeros(3)


    def target_tracking(self):
        """
        Target tracking component
        """
        if len(self.target_list)>0:
            last = len(self.target_list) - 1
            target = self.target_list[last]
            self.target_pxyz = np.array([
                target.position.x,
                target.position.y,
                target.position.z])
            self.target_psi = quaternion2psi([
                target.orientation.x,
                target.orientation.y,
                target.orientation.z,
                target.orientation.w])
            TargetDiff = self.target_pxyz - self.state_pxyz
            NormTargetDiff = np.linalg.norm(TargetDiff)
            TargetDiff /= NormTargetDiff
            TargetDiff *= SigmoidLike(NormTargetDiff, self.R_trg, self.d_trg)
            return (TargetDiff*self.freq)
        else:
            return np.zeros(3)


    def friend_target_tracking(self):
        """
        """
        self.target_pxyz = self.target_list[0]
        self.target_psi = quaternion2psi(self.target_list[1])
        TargetDiff = self.target_pxyz - self.state_pxyz
        NormTargetDiff = np.linalg.norm(TargetDiff)
        #print(NormTargetDiff)
        TargetDiff /= NormTargetDiff
        TargetDiff *= SigmoidLike(NormTargetDiff, self.R_trg, self.d_trg)
        return (TargetDiff*self.freq)


    def run(self):
        """
        """
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            rate.sleep()
            self.step()


    def choose_neighbors(self):
        """
        """
        for i in range(len(self.power_list)-self.nmax):
            ind = self.power_list.index(max(self.power_list)) # If distance is the parameter
            #ind = self.power_list.index(min(self.power_list)) # If power of communication is the parameter
            del self.total_accel[ind]
            del self.power_list[ind]
            del self.friend_list[ind]


    def update_forces(self):
        """
        """
        if len(self.msgs_uwb) > self.num_neighbors:
            for data in self.msgs_uwb[-self.num_neighbors:]:
                self.friend_state_pxyz = np.array([
                    data.friend_odom.pose.pose.position.x,
                    data.friend_odom.pose.pose.position.y,
                    data.friend_odom.pose.pose.position.z])
                self.friend_state_quat = np.array([
                    data.friend_odom.pose.pose.orientation.x,
                    data.friend_odom.pose.pose.orientation.y,
                    data.friend_odom.pose.pose.orientation.z,
                    data.friend_odom.pose.pose.orientation.w])
                self.friend_state_twist = np.array([
                    data.friend_odom.twist.twist.linear.x,
                    data.friend_odom.twist.twist.linear.y,
                    data.friend_odom.twist.twist.linear.z])
                self.essid = data.essid
                self.friend_essid = data.friend_essid
                self.distance = data.distance
                self.receivd_pwd = data.receivd_pwd
                # Agent Speed
                agent_speed = rotatewithquaternion(self.state_vabc, self.state_quat)
                # Update forces
                for data in self.laser_ranges:
                    if not np.isinf(data):
                        self.inf_count += 1
                alignment = self.align()
                cohesion = self.attraction()
                separate = self.repulsion()
                shill = self.shill()
                #print("Shill state: {}, {}".format(self.essid,shill))
                self.inf_count = 0
                target = self.target_tracking()
                #print("Shill state: {}, {}".format(self.essid,mydata))
                # Add speeds
                desired_speed = alignment + cohesion + separate + shill + target + (agent_speed/np.linalg.norm(agent_speed))*self.V_flock_l
                desired_speed = (desired_speed/np.linalg.norm(desired_speed)) * min(np.linalg.norm(desired_speed),self.max_speed)
                # Difference with agent speed
                speed_diff = desired_speed - agent_speed
                # Accel computation
                accel_limit = np.linalg.norm(speed_diff)
                self.state_accel = (speed_diff/accel_limit) * min(accel_limit,self.borne_acc_xy)
                # Add to different list
                self.friend_list.append(self.friend_essid)
                self.total_accel.append(self.state_accel)
                self.power_list.append(self.distance)
        else:
            pass


    def obstacle_detection(self):
        """
        Obstacle detection
        """
        if(len(self.laser_ranges)>1):
            # Number of beam per level (3 vertical, 12 horizontal). self.laser_ranges is built per vertical level. See gazebo_model/intelaero_bugwright/sensors.xacro
            ray_count_v = 3
            width = int(round(len(self.laser_ranges)/ray_count_v))
            # 12 first number are the 12 angles beams of first vertical beam (level) etc...
            # Split each level, find the min (closest dist to obst) and argmin (angle with obst). Multiplication by 4.92 (or 15) as the detector covers 59° (or 180°) with 12 beams.
            height_0_dist,height_0_angle = min(min(self.laser_ranges[0:width]),self.detector_range) , np.argmin(self.laser_ranges[0:width])
            height_1_dist,height_1_angle = min(min(self.laser_ranges[width:2*width]),self.detector_range) , np.argmin(self.laser_ranges[width:2*width])
            height_2_dist,height_2_angle = min(min(self.laser_ranges[2*width:]),self.detector_range) , np.argmin(self.laser_ranges[2*width:])
            self.obstacle = {height_0_dist:height_0_angle*15,height_1_dist:height_1_angle*15,height_2_dist:height_2_angle*15}  #180 degrees
            #self.obstacle = {height_0_dist:height_0_angle*4.92,height_1_dist:height_1_angle*4.92,height_2_dist:height_2_angle*4.92}  #59 degrees
            #print("Essid: {}, Obstacle: {}".format(self.essid,self.obstacle))
        else:
            self.obstacle = []
            pass


    def step(self):
        """
        """
        if self.state_ok:
            if len(self.power_list) < self.num_neighbors:
                #print("Essid: {}".format(self.essid))
                self.obstacle_detection()
                self.update_forces()
                    #print("Drone: {}, Aggregation Mode".format(self.essid))
            else:
                self.choose_neighbors()
                #self.obstacle_detection()
                #self.update_target()
                self.compute_cmd()
                #print("Essid: {}, Friend list: {}".format(self.essid,self.laser_ranges[0]))
                self.total_accel = []
                self.msgs_uwb = []
                self.power_list = []
                self.friend_list = []
                self.laser_ranges = []
                self.obstacle = []
                self.count+=1
                #print("Essid: {}, Count: {}".format(self.essid,self.count))


    def compute_cmd(self):
        """
        """
        # Current attitude
        state_psi, state_tet, state_phi = quaternion2euler(self.state_quat)
        # Position errors
        error_xyz = self.target_pxyz - self.state_pxyz
        # Linear velocity errors
        error_vxyz = self.target_vxyz - rotatewithquaternion(self.state_vabc, self.state_quat)
        # Default command
        cmd_axyz = [0,0,G]

        #if self.first_round:
        # if self.laser_time < 2.0:
        #     #cmd_axyz = [2*np.random.uniform(-1,1),2*np.random.uniform(-1,1),np.random.uniform(-1,1)+G]
        #     #cmd_axyz = [5*np.random.rand(1)[0],3*np.random.rand(1)[0],G]
        #     cmd_axyz = [0,0,G]
        #     #self.first_round = False
        #     cmd_ax1y1z = rotatewithpsi(cmd_axyz, -state_psi)
        #     print("Essid: {}, Random Mode".format(self.essid))
        cmd_axyz = 2*sum(self.total_accel) # The multiplication is to increase speed in simulation
        cmd_axyz[2] = 2.0*error_vxyz[2] + error_xyz[2]
        cmd_axyz[2] = max(-self.borne_acc_z, min(self.borne_acc_z, cmd_axyz[2]))
        cmd_axyz[2] += G

        # Acceleration commands in FLU(t) frame
        cmd_ax1y1z = rotatewithpsi(cmd_axyz, -state_psi)
        # Attitude commands <=> Acceleration commands
        cmd_tet = math.atan2(cmd_ax1y1z[0], cmd_ax1y1z[2])
        cmd_phi = math.atan2(-cmd_ax1y1z[1], cmd_ax1y1z[2]/math.cos(cmd_tet))
        cmd_qat = euler2quaternion(self.target_psi, cmd_tet, cmd_phi)
        #print('Target_psi:{}'.format(self.target_psi))
        # Thrust command
        cmd_thrust = rotatewithquaternion(cmd_axyz, quaternioninv(self.state_quat))[2]
        # Attitude error
        err_psi = 2.0*math.atan(math.tan(0.5*(self.target_psi - state_psi)))
        err_tet = cmd_tet - state_tet
        err_phi = cmd_phi - state_phi
        # Derivative command in attitude
        self.target_dpsi = (self.target_psi - self.cmd_psi_prev) * self.freq
        cmd_dtet = (cmd_tet - self.cmd_tet_prev) * self.freq
        cmd_dphi = (cmd_phi - self.cmd_phi_prev) * self.freq
        self.cmd_psi_prev = self.target_psi
        self.cmd_tet_prev = cmd_tet
        self.cmd_phi_prev = cmd_phi
        # Rotation transfer inverse matrix
        Ti = np.ndarray(
             shape=(3, 3),
             dtype=float,
             buffer=np.array(
                 [0.0, -math.sin(state_psi),  math.cos(state_tet)*math.cos(state_psi),
                  0.0,  math.cos(state_psi),  math.cos(state_tet)*math.sin(state_psi),
                  1.0,  0.0,                 -math.sin(state_tet)]))
        # Angular velocity command
        cmd_dA = np.ndarray(
             shape=(3, 1),
             dtype=float,
             buffer=np.array(
                [self.target_dpsi + err_psi,
                 cmd_dtet + err_tet,
                 cmd_dphi + err_phi]))
        cmd_wabc = rotatewithquaternion(np.matmul(Ti, cmd_dA).flatten(), quaternioninv(self.state_quat))
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
        return cmd_axyz



    # Callback functions -----------------------------------------------
    def callback_odometry(self, data):
        """
        """
        # Position in ENU frame
        self.state_pxyz = np.array([
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
        self.state_vabc = np.array([
            data.twist.twist.linear.x,
            data.twist.twist.linear.y,
            data.twist.twist.linear.z])
        # Angular velocity in FLU(t) frame
        self.state_wabc = np.array([
            data.twist.twist.angular.x,
            data.twist.twist.angular.y,
            data.twist.twist.angular.z])
        # OK state
        self.state_ok = True


    def callback_target(self,data):
        """
        """
        self.target_list.append(data)


    def callback_uwb(self,data):
        """
        """
        self.msgs_uwb.append(data)


    def callback_imu(self,data):
        """
        """
        self.state_accel = np.array([
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z])


    def callback_oa(self,data):
        """
        """
        self.laser_ranges = data.ranges
        self.laser_time = data.header.stamp.secs


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
    rospy.init_node("flocking_pilot", anonymous=False)
    pilote = DroneFlockingPilot()
    pilote.run()
