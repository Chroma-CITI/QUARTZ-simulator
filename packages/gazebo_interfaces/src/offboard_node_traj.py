#!/usr/bin/env python3
# coding: utf-8


"""
 * File: offboard_node_traj.py
 * Stack and tested in Gazebo Classic 11 SITL
"""

import rospy
import tf2_ros

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_geometry_msgs import do_transform_vector3, do_transform_pose # Has to be used with a PoseStamped
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Transform, Twist, TwistStamped, Vector3Stamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

##########################  Helper functions ######################

def ConvertPoseToTrajectoryWaypoint(pose):
    convertPoint = MultiDOFJointTrajectoryPoint()
    convertPoint.transforms.append(Transform())
    convertPoint.transforms[0].translation.x = pose.position.x
    convertPoint.transforms[0].translation.y = pose.position.y
    convertPoint.transforms[0].translation.z = pose.position.z
    convertPoint.transforms[0].rotation = pose.orientation

    return convertPoint

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

def ConvertTrajectoryWaypointToPoseStamped(point, header):
    poseStamped = PoseStamped()
    poseStamped.header = header
    poseStamped.pose = ConvertTrajectoryWaypointToPose(point)
    return poseStamped

def DoTransformTrajectory(trajectory, sourceToTargetTransform):
    transformedTraj = MultiDOFJointTrajectory()
    new_frame = sourceToTargetTransform.header.frame_id
    for i, point in enumerate(trajectory.points):
        pointPose = ConvertTrajectoryWaypointToPoseStamped(point, transformedTraj.header)
        pointPoseInTargetFrame = do_transform_pose(pointPose, sourceToTargetTransform)
        transformedPoint = ConvertPoseToTrajectoryWaypoint(pointPoseInTargetFrame.pose)
        if len(point.velocities):
            transformedPoint.velocities.append(transform_twist(point.velocities[0], sourceToTargetTransform))
        transformedTraj.points.append(transformedPoint)
    transformedTraj.header.stamp = trajectory.header.stamp
    transformedTraj.header.frame_id = new_frame
    return transformedTraj

def CreateVizMarkerForTrajectory(trajectory):
    # Convert the trajectory to markers
    marker_array = MarkerArray()
    for i, point in enumerate(trajectory.points):
        for transform in point.transforms:
            marker = Marker()
            marker.header.frame_id = trajectory.header.frame_id
            marker.header.stamp = trajectory.header.stamp
            marker.ns = "trajectory"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose.position.x = transform.translation.x
            marker.pose.position.y = transform.translation.y
            marker.pose.position.z = transform.translation.z
            marker.pose.orientation = transform.rotation

            marker.scale.x = 0.1
            marker.scale.y = 0.2
            marker.scale.z = 0.3

            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

            marker_array.markers.append(marker)
    return marker_array

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

def HoveringTrajectory(targetFrame="world"):
    # Define a transform 1 m above the original frame
    transform = Transform()
    transform.translation.z = 1.0
    transform.rotation.w = 1.0
    # Add this transform to a trajectory point
    point = MultiDOFJointTrajectoryPoint()
    point.transforms.append(transform)
    # Finally, create a trajectory with this point
    trajectory = MultiDOFJointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = targetFrame
    trajectory.points.append(point)
    rospy.loginfo("Hovering trajectory: %r", trajectory)
    return trajectory

def transform_twist(twist_in, transform):
    # Apply the rotation from the transform to the linear velocity
    linear_twist = Vector3Stamped()
    linear_twist.vector = twist_in.linear
    linear_out = rotate_vector(linear_twist, transform.transform.rotation)

    # Apply the rotation from the transform to the angular velocity
    angular_twist = Vector3Stamped()
    angular_twist.vector = twist_in.angular
    angular_out = rotate_vector(angular_twist, transform.transform.rotation)

    # Create a new Twist message with the transformed velocities
    twist_out = Twist()
    twist_out.linear = linear_out
    twist_out.angular = angular_out

    return twist_out

def rotate_vector(vector, rotation):
    # Convert the rotation quaternion to a matrix using tf2
    transform = TransformStamped()
    transform.transform.rotation = rotation
    
    # Use tf2_geometry_msgs to rotate the vector
    rotated_vector = do_transform_vector3(vector, transform)

    return rotated_vector.vector

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
        self._currentTrajectory = HoveringTrajectory()
        self._lastRequest = rospy.Time.now()

        # Handle frames and transforms
        self._worldFrame = rospy.get_param('~worldFrame', 'world') # Frame in which the trajectory and the odometry is expressed
        self._localFrame = rospy.get_param('~localFrame', 'map')  # The local frame of the drone, usually the map
        rospy.loginfo("Received the local frame of the drone which is %s", self._localFrame)
        rospy.loginfo("Received the world frame which is %s", self._worldFrame)
        self._world_to_local_static_transform = ComputeLastestTransform(self._worldFrame, self._localFrame)
        rospy.loginfo("transform from world to local frame is %r", self._world_to_local_static_transform)

        # Initialize the ros subscribers
        rospy.Subscriber("mavros/state", State, callback = self._MavrosStateCallback)
        rospy.Subscriber("command/trajectory", MultiDOFJointTrajectory, self._TrajectoryCallback)

        # Initialize the ros publishers
        # self._localPosePub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self._localTrajPub = rospy.Publisher("mavros/setpoint_trajectory/local", MultiDOFJointTrajectory, queue_size=10)
        self._trajectoryVizPub = rospy.Publisher("trajectory_viz_marker", MarkerArray, queue_size=10)
        self._velocitiesPub = rospy.Publisher("trajectory_viz_vel", TwistStamped, queue_size=10)
        rospy.loginfo("Finishing sub and pub set up")

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
            self._SendTarget()

            self._rate.sleep()


    def _MavrosStateCallback(self, msg):
        self._currentState = msg
    
    def _TrajectoryCallback(self, msg):
        # Check if the received trajectory is valid
        if msg is None or not msg.points:
            return

        # Reset current trajectory
        self._ResetTargets()
        # Update the current trajectory and target pose
        self._currentTrajectory = msg


    def _ResetTargets(self):
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

    def _SendTarget(self):
        transformedTraj = DoTransformTrajectory(self._currentTrajectory, self._world_to_local_static_transform)
        self._localTrajPub.publish(transformedTraj)
        self._PublishTrajectoryViz(transformedTraj)

    def _PublishTrajectoryViz(self, trajectory):
        self._trajectoryVizPub.publish(CreateVizMarkerForTrajectory(trajectory))
        for i, point in enumerate(trajectory.points):
            if point.velocities:
                twistStamped = TwistStamped()
                twistStamped.header = trajectory.header
                twistStamped.header.seq = i
                twistStamped.twist = point.velocities[0]
                self._velocitiesPub.publish(twistStamped)

if __name__ == '__main__':
    try:
        OffboardController().Spin()  
    except rospy.ROSInterruptException:
        pass
