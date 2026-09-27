#!/usr/bin/env python3
# coding: utf-8


"""
 * File: offboard_node_traj.py
 * Stack and tested in Gazebo Classic 11 SITL
"""

import math
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler

from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from geometry_msgs.msg import Vector3

from tf2_geometry_msgs import do_transform_vector3, do_transform_pose # Has to be used with a PoseStamped
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Transform, Twist, Vector3Stamped, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

##########################  Helper functions  ######################

def is_valid_quaternion(q, tolerance=1e-3):
    components = [q.x, q.y, q.z, q.w]

    # Check for NaNs or Infs
    if any(math.isnan(c) or math.isinf(c) for c in components):
        return False

    # Check if it's approximately normalized
    norm = math.sqrt(sum(c ** 2 for c in components))
    return abs(norm - 1.0) <= tolerance

def ConvertPoseToTrajectoryWaypoint(pose):
    convertPoint = MultiDOFJointTrajectoryPoint()
    convertPoint.transforms.append(Transform())
    convertPoint.transforms[0].translation.x = pose.position.x
    convertPoint.transforms[0].translation.y = pose.position.y
    convertPoint.transforms[0].translation.z = pose.position.z
    convertPoint.transforms[0].rotation = pose.orientation

    return convertPoint

def ConvertTrajectoryWaypointToPoseAndVelocity(point):
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

    return convertPose, point.velocities[0]

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

def ComputeRosWorldToGazeboWorldTransform():
    static_transform_stamped = TransformStamped()

    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "world" # gazebo
    static_transform_stamped.child_frame_id = "world" # ros
    # No translation
    static_transform_stamped.transform.translation.x = 0.0
    static_transform_stamped.transform.translation.y = 0.0
    static_transform_stamped.transform.translation.z = 0.0
    # -90 degrees (pi/2) around Z-axis
    quat = quaternion_from_euler(0, 0, -math.pi / 2)
    static_transform_stamped.transform.rotation.x = quat[0]
    static_transform_stamped.transform.rotation.y = quat[1]
    static_transform_stamped.transform.rotation.z = quat[2]
    static_transform_stamped.transform.rotation.w = quat[3]

    return static_transform_stamped

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

###################################################################

class SimulationController:
    """ In charge of converting a drone's pose target from a ros message to a mavros message.
    It expects the ros's pose target to be expressed in a given reference frame and convert it to the drone's local frame.
    """

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('simulation_controller', anonymous=True)

        rospy.Subscriber("command/trajectory", MultiDOFJointTrajectory, self._TrajectoryCallback)
        self._currentTrajectory = None
        self._lastPose = Pose()
        self._lastPose.position.x = rospy.get_param("~target_px", 0.0)
        self._lastPose.position.y = rospy.get_param("~target_py", 0.0)
        self._lastPose.position.z = rospy.get_param("~target_pz", 2.5) # Hovering 1 m above the ground by default

        # Wait for services
        rospy.wait_for_service('/gazebo/get_physics_properties')
        rospy.wait_for_service('/gazebo/set_physics_properties')
        rospy.wait_for_service('/gazebo/set_model_state')

        self._callGetPhysicsClient = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        self._callSetPhysicsClient = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self._callSetModelStateClient = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # UAV id TODO: fix parameter acquisition
        self._model_id = rospy.get_param('~model_id', "iris_velodyne_0")

        # Transform from ros world to gazebo world (90 degree rotation around the z-axis)
        self._world_to_local_static_transform = ComputeRosWorldToGazeboWorldTransform()

        # Setpoint frequency should be quite fast
        frequency = rospy.get_param('~freq', 50)
        self._rate = rospy.Rate(frequency)

        # Loop until service is properly called
        rospy.loginfo("Try calling set physics service...")
        is_gravity_disabled = False
        while(not is_gravity_disabled):
            is_gravity_disabled = self._DisableGravity()
        rospy.loginfo("Gravity is disabled.")


    def Spin(self):
        while(not rospy.is_shutdown()):
            self._PubTrajectory()
            self._rate.sleep()
    
    def _TrajectoryCallback(self, msg):
        # Check if the received trajectory is valid
        if msg is None or not msg.points:
            return

        # Update the current trajectory and target pose
        self._currentTrajectory = DoTransformTrajectory(msg, self._world_to_local_static_transform)

    def _PubTrajectory(self):
        if self._currentTrajectory and self._currentTrajectory.points:
            last_waypoint = self._currentTrajectory.points.pop(0)
            self._lastPose, velocity = ConvertTrajectoryWaypointToPoseAndVelocity(last_waypoint)
            self._PubPose(self._lastPose, velocity)
        elif self._lastPose:
            self._PubPose(self._lastPose)

    def _PubPose(self, pose, velocity=Twist()):
        state_msg = ModelState()
        state_msg.model_name = self._model_id

        # Position
        state_msg.pose = pose
        if not is_valid_quaternion(pose.orientation):
            rospy.logwarn("Invalid quaternion: %r ", pose.orientation)
            pose.orientation = Quaternion()
        # Velocity
        state_msg.twist = velocity

        # Apply new state
        try:
            resp = self._callSetModelStateClient(state_msg)
            if not resp.success:
                rospy.logwarn("Failed to teleport UAV.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    def _DisableGravity(self):
        # Get current physics properties
        current_physics = self._callGetPhysicsClient()

        # Modify gravity vector to zero
        zero_gravity = Vector3(0.0, 0.0, 0.0)

        # Set the updated physics properties
        try:
            self._callSetPhysicsClient(
                time_step=current_physics.time_step,
                max_update_rate=current_physics.max_update_rate,
                gravity=zero_gravity,
                ode_config=current_physics.ode_config
            )
            rospy.loginfo("Gravity successfully disabled in Gazebo.")
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Failed to disable gravity: %s", e)
            return False

if __name__ == '__main__':
    try:
        SimulationController().Spin()  
    except rospy.ROSInterruptException:
        pass
