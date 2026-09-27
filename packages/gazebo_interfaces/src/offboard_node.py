#!/usr/bin/env python3
# coding: utf-8


"""
 * File: offboard_node.py
 * Stack and tested in Gazebo Classic 11 SITL
"""

import rospy
import numpy
import tf2_ros
from math_utils import D2R, quaternion2euler

from nav_msgs.msg import Odometry
from tf2_geometry_msgs import do_transform_pose # Has to be used with a PoseStamped
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

##########################  Helper functions ######################

def ConvertTrajectoryWaypointToPose(point):
    """Convert a MultiDOFJointTrajectoryPoint to a Pose
    Cf https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html
    
    Args:
        point (MultiDOFJointTrajectoryPoint): a trajectory point is composed of transforms for each of its DOF (here 1), a velocity and an acceleration.
    """
    pointTransform = point.transforms[0] # Because we only have 1DOF here
    convertPose = Pose()
    convertPose.position.x = pointTransform.translation.x
    convertPose.position.y = pointTransform.translation.y
    convertPose.position.z = pointTransform.translation.z
    convertPose.orientation = pointTransform.rotation

    return convertPose

def angle_difference(angle1, angle2):
    """Computes the shortest difference between two angles in radians."""
    return (angle2 - angle1 + numpy.pi) % (2 * numpy.pi) - numpy.pi

def ComputeLastestTransform(sourceFrame, targetFrame):
    # Initialize the TF2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    targetPoseFrameToLocalFrameTransform = TransformStamped()

    # If the transform between the source and the dest frame exist, compute it
    try:
        # Wait for the transform from the trajectoryFrameId to the _localFrame to be available
        targetPoseFrameToLocalFrameTransform = tf_buffer.lookup_transform(
            targetFrame,  # Target frame: the frame in which the pose will be transform to
            sourceFrame,  # Source frame: the frame in which the pose will be transform from
            rospy.Time(0),  # Get the latest available transform
            rospy.Duration(1.0)  # Timeout duration in s
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"Transform failed: {e}")

    return targetPoseFrameToLocalFrameTransform

###################################################################

class OffboardController:
    """ In charge of converting a drone's pose target from a ros message to a mavros message.
    It expects the ros's pose target to be expressed in a given reference frame and convert it to the drone's local frame.
    """

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('offboard_controller', anonymous=True)

        # Initialize class attributes
        self._currentState = State()
        self._currentOdometry = Odometry() # expressed in the odometric frame
        self._currentTargetPose = PoseStamped()  # i.e. the drone should not move initially
        self._currentTargetPose.pose.position.z = 1.0 # After takeoff, the drone will hover 1 m above its initial pose (usually the ground)
        self._currentTrajectory = MultiDOFJointTrajectory()
        self._lastRequest = rospy.Time.now()

        # Handle frames and transforms
        self._worldFrame = rospy.get_param('~worldFrame', 'world') # Frame in which the trajectory and the odometry is expressed
        self._localFrame = rospy.get_param('~localFrame', 'map')  # The local frame of the drone, usually the map
        rospy.loginfo("Received the local frame of the drone which is %s", self._localFrame)
        rospy.loginfo("Received the world frame which is %s", self._worldFrame)
        self._world_to_local_static_transform = ComputeLastestTransform(self._worldFrame, self._localFrame)
        rospy.loginfo("transform from world to local frame is %r", self._world_to_local_static_transform)

        # Get navigation parameters
        self._distanceThreshold = rospy.get_param('~distanceThreshold', 1.0) # in m
        self._yawThreshold = rospy.get_param('~yawThreshold', 10.0) # in degrees

        # Initialize the ros subscribers
        rospy.Subscriber("mavros/state", State, callback = self._MavrosStateCallback)
        rospy.Subscriber("mavros/global_position/local", Odometry, self._OdometryCallback)
        rospy.Subscriber("command/trajectory", MultiDOFJointTrajectory, self._TrajectoryCallback)

        # Initialize the ros publishers
        self._localPosePub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Wait for the mavros services and initialize clients
        rospy.wait_for_service("mavros/cmd/arming")
        self._armingClient = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("mavros/set_mode")
        self._settingModeClient = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Create parameters for the service's request calls
        self._offboardModeRequest = SetModeRequest()
        self._offboardModeRequest.custom_mode = 'OFFBOARD'

        self._armCmdRequest = CommandBoolRequest()
        self._armCmdRequest = True

        # Setpoint publishing MUST be faster than 2Hz for px4
        self._rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self._currentState.connected):
            self._rate.sleep()


    def Spin(self):
        self._lastRequest = rospy.Time.now()
        while(not rospy.is_shutdown()):
            # Ensure that the offboard mode signal and the arming command is sent regularly
            # (cf https://docs.px4.io/main/en/flight_modes/offboard.html)
            self._EnableOffboardModeAndArmVehicle()
            # Update the goal if it is reached and send it to mavros
            self._UpdateGoal()
            self._SendTarget()

            self._rate.sleep()


    def _MavrosStateCallback(self, msg):
        self._currentState = msg

    def _OdometryCallback(self, msg):
        self._currentOdometry = msg
    
    def _TrajectoryCallback(self, msg):
        # Check if the received trajectory is valid
        if msg is None or not msg.points:
            return

        # Reset current trajectory
        self._ResetTargets()

        # Update the current trajectory and target pose
        self._currentTrajectory = msg
        self._currentTargetPose = self._ConvertTrajectoryWaypointToTargetPose(
            self._currentTrajectory.points.pop(0),
            self._currentTrajectory.header.frame_id
        )

    def _ConvertTrajectoryWaypointToTargetPose(self, waypoint, trajectoryFrameId):
        targetPoseInTrajectoryFrame = PoseStamped()
        targetPoseInTrajectoryFrame.header.frame_id = self._localFrame
        targetPoseInTrajectoryFrame.header.stamp = rospy.Time.now()
        targetPoseInTrajectoryFrame.pose = ConvertTrajectoryWaypointToPose(waypoint) # Expressed in the same frame as the trajectory

        return targetPoseInTrajectoryFrame

    def _ResetTargets(self):
        self._currentTargetPose = PoseStamped()
        if self._currentOdometry:
            # when resetting the target keep the current pose as the target pose
            self._currentTargetPose = PoseStamped()
            self._currentTargetPose.header.stamp = rospy.Time.now()
            self._currentTargetPose.header.frame_id = self._worldFrame
            self._currentTargetPose.pose = self._currentOdometry.pose.pose
        self._currentTrajectory = MultiDOFJointTrajectory()

    def _EnableOffboardModeAndArmVehicle(self):
        if(self._currentState.mode != "OFFBOARD" and (rospy.Time.now() - self._lastRequest) > rospy.Duration(5.0)):
            if(self._settingModeClient.call(self._offboardModeRequest).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            self._lastRequest = rospy.Time.now()
        else:
            if(not self._currentState.armed and (rospy.Time.now() - self._lastRequest) > rospy.Duration(5.0)):
                if(self._armingClient.call(self._armCmdRequest).success == True):
                    rospy.loginfo("Vehicle armed")

                self._lastRequest = rospy.Time.now()

    def _UpdateGoal(self):
        # Check if the current trajectory and last odometry are available
        if not self._currentOdometry or not self._currentTargetPose:
            return
        
        # Compute the distance difference between the target and the current pose
        targetPosition = numpy.array([
            self._currentTargetPose.pose.position.x,
            self._currentTargetPose.pose.position.y,
            self._currentTargetPose.pose.position.z,
        ])
        currentPosition = numpy.array([
            self._currentOdometry.pose.pose.position.x,
            self._currentOdometry.pose.pose.position.y,
            self._currentOdometry.pose.pose.position.z,
        ])
        distanceError = numpy.linalg.norm(targetPosition - currentPosition)

        # Compute the heading (yaw) difference between the target and the current pose
        targetQuaternion = [
            self._currentTargetPose.pose.orientation.x,
            self._currentTargetPose.pose.orientation.y,
            self._currentTargetPose.pose.orientation.z,
            self._currentTargetPose.pose.orientation.w
        ]
        targetYaw = quaternion2euler(targetQuaternion)[2]
        currentQuaternion = [
            self._currentOdometry.pose.pose.orientation.x,
            self._currentOdometry.pose.pose.orientation.y,
            self._currentOdometry.pose.pose.orientation.z,
            self._currentOdometry.pose.pose.orientation.w
        ]
        currentYaw = quaternion2euler(currentQuaternion)[2]
        yawError = angle_difference(targetYaw, currentYaw)

        # Check if the current waypoint is reached
        if (distanceError < self._distanceThreshold) and (yawError < self._yawThreshold * D2R):
            if self._currentTrajectory and self._currentTrajectory.points:
                # Update current target pose
                self._currentTargetPose = self._ConvertTrajectoryWaypointToTargetPose(
                    self._currentTrajectory.points.pop(0),
                    self._currentTrajectory.header.frame_id
                )
            else:
                rospy.loginfo_throttle_identical(60, "No more points in the trajectory, keeping the last waypoint as the current target.")

    def _SendTarget(self):
        # Transform the pose so that it is expressed in the local frame of the drone (usually its map frame)
        targetPoseInLocalFrame = do_transform_pose(self._currentTargetPose, self._world_to_local_static_transform)
        self._localPosePub.publish(targetPoseInLocalFrame)

if __name__ == '__main__':
    try:
        OffboardController().Spin()  
    except rospy.ROSInterruptException:
        pass
