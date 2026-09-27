#!/usr/bin/env python3
# coding: utf-8

import rospy
import itertools
import numpy as np

from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Twist, Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from pointcloud_helper import convert_pointcloud2_to_array, convert_array_to_pointcloud2, select_points_in_sub_sphere

################################ Helper Function ######################################

def line_intersects_aabb(p0, d, min_b, max_b):
    with np.errstate(divide='ignore'):
        t1 = (min_b - p0) / d
        t2 = (max_b - p0) / d

    tmin = np.max(np.minimum(t1, t2))
    tmax = np.min(np.maximum(t1, t2))

    # Check tmin so that only the aabox in the direction of the vector is selected
    return tmax >= max(tmin, 0)


def VisualiseDirectionVector(vector, frame_id="world"):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = "direction_vector"
    m.id = 0
    m.type = Marker.ARROW
    m.action = Marker.ADD

    # Arrow start and end points
    origin = [0.0, 0.0, 0.0]   # starting point of the vector in frame_id
    m.points.append(Point(origin[0], origin[1], origin[2]))
    m.points.append(Point(origin[0] + vector[0],
                          origin[1] + vector[1],
                          origin[2] + vector[2]))

    # Arrow thickness
    m.scale.x = 0.05  # shaft diameter
    m.scale.y = 0.1   # head diameter
    m.scale.z = 0.1   # head length

    # Color
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.color.a = 1.0  # opacity

    return m

################################ Helper Class ######################################

class BoundingBox:
    def __init__(self, min_dims: np.array, max_dims: np.array):
        self._min_dims = min_dims
        self._max_dims = max_dims
    
    def Get_min_dims(self):
        return self._min_dims
    
    def Get_max_dims(self):
        return self._max_dims
    
    def Expand_bounding_box(self, expansion):
        return self._min_dims.copy(), self._max_dims.copy()
    
    def Print(self):
        return (
            f"bb: \t"
            f" - min dims: {self._min_dims}\t"
            f" - max dims: {self._max_dims}\t"
        )

    def Create_bounding_box_marker_expanded(self, frame_id="world", marker_id=1, expansion=0.0, color=ColorRGBA(1, 0, 0, 0.3)):
        min_dims, max_dims = self.Expand_bounding_box(expansion)
        return self.Create_bounding_box_marker(frame_id, marker_id, min_dims, max_dims, color)

    def Create_bounding_box_marker(self, frame_id="world", marker_id=1, min_dims=None, max_dims=None, color=ColorRGBA(1, 0, 0, 0.3)):
        min_dims = min_dims if min_dims is not None else self._min_dims
        max_dims = max_dims if max_dims is not None else self._max_dims

        center = (min_dims + max_dims) / 2.0
        scale = (max_dims - min_dims)

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD

        m.pose.position.x = center[0]
        m.pose.position.y = center[1]
        m.pose.position.z = center[2]

        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1

        m.scale.x = scale[0]
        m.scale.y = scale[1]
        m.scale.z = scale[2]

        # Color: semi-transparent
        m.color = color

        return m

class FrontBoundingBox(BoundingBox):
    def __init__(self, radius, dim):
        self._min_dims = np.array([ -radius, radius, -radius])
        self._max_dims = np.array([ radius, dim, radius])

    def Expand_bounding_box(self, expansion):
        a = self._max_dims.copy()
        a[1] += expansion
        return self._min_dims.copy(), a
    
class BackBoundingBox(BoundingBox):
    def __init__(self, radius, dim):
        self._min_dims = np.array([ -radius, -dim, -radius])
        self._max_dims = np.array([ radius, -radius, radius])

    def Expand_bounding_box(self, expansion):
        a = self._min_dims.copy()
        a[1] -= expansion
        return a, self._max_dims.copy()
class TopBoundingBox(BoundingBox):
    def __init__(self, radius, dim):
        self._min_dims = np.array([ -radius, -radius, radius])
        self._max_dims = np.array([ radius, radius, dim])

    def Expand_bounding_box(self, expansion):
        a = self._max_dims.copy()
        a[2] += expansion
        return self._min_dims.copy(), a
    
class BottomBoundingBox(BoundingBox):
    def __init__(self, radius, dim):
        self._min_dims = np.array([ -radius, -radius, -dim])
        self._max_dims = np.array([ radius, radius, -radius])

    def Expand_bounding_box(self, expansion):
        a = self._min_dims.copy()
        a[2] -= expansion
        return a, self._max_dims.copy()
class RightBoundingBox(BoundingBox):
    def __init__(self, radius, dim):
        self._min_dims = np.array([ radius, -radius, -radius])
        self._max_dims = np.array([ dim, radius, radius])

    def Expand_bounding_box(self, expansion):
        a = self._max_dims.copy()
        a[0] += expansion
        return self._min_dims.copy(), a
    
class LeftBoundingBox(BoundingBox):
    def __init__(self, radius, dim):
        self._min_dims = np.array([ -dim, -radius, -radius])
        self._max_dims = np.array([ -radius, radius, radius])

    def Expand_bounding_box(self, expansion):
        a = self._min_dims.copy()
        a[0] -= expansion
        return a, self._max_dims.copy()

########################################################################################


class CollisionChecker:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('collision_checker', anonymous=True)

        # Get parameters
        self._debug = rospy.get_param('/debug', True)
        self._safety_factor = rospy.get_param('safety_factor', 4.0)
        self._min_number_of_collision_points = rospy.get_param('min_number_of_collision_points', 10)
        self._max_expected_speed = rospy.get_param('/mav_local_planner/v_max', 5.0) # In m /s 
        self._check_frequency = rospy.get_param('check_frequency', 20) # In Hz
        self._robot_radius = rospy.get_param('/mav_local_planner/robot_radius', 0.5) # In m

        # drone_frame_id = rospy.get_param('drone_frame_id')
        odometry_topic = rospy.get_param('~odometry_topic')
        pointcloud_topic = rospy.get_param('~pointcloud_topic')
        emergency_stop_client_topic = rospy.get_param('~emergency_stop_client_topic')

        # Set node attributes
        self._check_bounding_boxes = self.GenerateCheckBoundingBoxes()
        self._check_min_bounding_box = self.GenerateCheckMinBoundingBox()
        self._current_pose = Pose()
        self._current_Twist = Twist()

        # Subscribers
        self._odometry_subscriber = rospy.Subscriber(odometry_topic, Odometry, self.OdometryCallback)
        self._pointcloud_subscriber = rospy.Subscriber(pointcloud_topic, PointCloud2, self.PointcloudCallback)

        # Debug publisher
        self._collision_pointcloud_publisher = rospy.Publisher("collision_pointcloud", PointCloud2, queue_size=1)
        self._before_collision_pointcloud_publisher = rospy.Publisher("before_collision_pointcloud", PointCloud2, queue_size=1)
        self._collision_bbox_publisher = rospy.Publisher("collision_bbox_marker_array", MarkerArray, queue_size=1)

        # Emergency stop client (TODO: Is this the best way?)
        rospy.loginfo("[collision_checker] Waiting for service %s", emergency_stop_client_topic)
        rospy.wait_for_service(emergency_stop_client_topic)
        self._emergency_stop_client = rospy.ServiceProxy(emergency_stop_client_topic, Empty)

    def GenerateCheckMinBoundingBox(self):
        min_dim = 0.7 * self._robot_radius
        return [
            BoundingBox(
                np.array([ -min_dim, -min_dim, -min_dim]),   # min dimensions
                np.array([ min_dim, min_dim, min_dim])    # max dimensions
            )
        ]

    def GenerateCheckBoundingBoxes(self):
        # Computing the distance ahead to check for collision
        check_interval_in_s = 1.0 / self._check_frequency
        self._computed_distance_ahead_to_check = self._max_expected_speed * check_interval_in_s * self._safety_factor
        rospy.logwarn("[collision_checker] computed_distance_ahead_to_check =  %f m and self._robot_radius = %f m", self._computed_distance_ahead_to_check, self._robot_radius)

        dim = self._robot_radius + self._computed_distance_ahead_to_check
        radius = self._robot_radius

        return [
            FrontBoundingBox(radius, dim),
            BackBoundingBox(radius, dim),
            TopBoundingBox(radius, dim),
            BottomBoundingBox(radius, dim),
            RightBoundingBox(radius, dim),
            LeftBoundingBox(radius, dim)
        ]

    def GetPointInCheckBoundingBoxes(self, drone_direction_vector, pointcloud, pointcloud_frame_id):
        bb_marker_array = MarkerArray()
        if self._debug:
            delete_array = MarkerArray()
            delete_array.markers.append(Marker(action = Marker.DELETEALL))
            # Publish marker to delete previous markers
            self._collision_bbox_publisher.publish(delete_array)
            # Display direction vector
            bb_marker_array.markers.append(VisualiseDirectionVector(drone_direction_vector, pointcloud_frame_id))

        if np.linalg.norm(drone_direction_vector) < self._robot_radius:
            available_bounding_boxes = list() # NOTE: used to be _check_min_bounding_box but too strict
        else:
            available_bounding_boxes = self._check_bounding_boxes

        mask = np.zeros(len(pointcloud), dtype=bool)
        for i, bb in enumerate(available_bounding_boxes):
            expansion = np.linalg.norm(drone_direction_vector) * (1.0 / self._check_frequency) * self._safety_factor
            min_dims, max_dims = bb.Expand_bounding_box(expansion)
            is_vector_in_bounding_box = line_intersects_aabb(np.array([0, 0, 0]), drone_direction_vector, min_dims, max_dims)
            # is_vector_in_bounding_box = np.all(drone_direction_vector >= min_dims) and np.all(drone_direction_vector <= max_dims)
            if self._debug:
                bb_marker_array.markers.append(bb.Create_bounding_box_marker(pointcloud_frame_id, i, min_dims, max_dims)) # Display bounding box
            if not is_vector_in_bounding_box:
                continue

            # Select points from the pointcloud inside the bounding box
            bb_mask = np.all((pointcloud >= min_dims) & (pointcloud <= max_dims), axis=1)
            # rospy.logwarn("[collision_checker] Detected %f points", bb_mask.sum())
            mask = mask | bb_mask

            if self._debug:
                bb_marker_array.markers.append(bb.Create_bounding_box_marker(pointcloud_frame_id, i, min_dims, max_dims, ColorRGBA(0, 1, 0, 0.6))) # Display bounding box
                # Display collision points
                self._before_collision_pointcloud_publisher.publish(convert_array_to_pointcloud2(pointcloud, rospy.Time.now(), pointcloud_frame_id))
                self._collision_pointcloud_publisher.publish(convert_array_to_pointcloud2(pointcloud[bb_mask], rospy.Time.now(), pointcloud_frame_id))

        if self._debug:
            self._collision_bbox_publisher.publish(bb_marker_array)
        return mask


    def CheckForCollision(self, pointcloud_array, pointcloud_frame_id="world"):
        time_before_check = rospy.Time.now()
        # Expecting the speed to be in m/s and the distance in m
        if not self._current_Twist:
            rospy.logwarn("[collision_checker] No speed has been received, cannot check for collision!!!")
            return
        
        # Use the speed to compute the direction of travel
        drone_direction_vector = np.array(
            [
                self._current_Twist.linear.x,
                self._current_Twist.linear.y,
                self._current_Twist.linear.z
            ]
        )

        # Collision check is fast, should be done near the observer
        # So reducing the pointcloud to a sub sphere of radius computed_min_distance_ahead for speedup
        sub_sphere_radius = self._robot_radius + np.linalg.norm(drone_direction_vector)
        reduced_pointcloud = pointcloud_array[select_points_in_sub_sphere(pointcloud_array, sub_sphere_radius)]

        # Select the points in the direction of travel (i.e. inside the bounding box)
        point_in_collision_mask = self.GetPointInCheckBoundingBoxes(drone_direction_vector, reduced_pointcloud, pointcloud_frame_id)

        # If points are found in the collision volume, cancel the planner goal
        if point_in_collision_mask.sum() > self._min_number_of_collision_points:
            # Trigger emergency stop
            self._emergency_stop_client.call()
            rospy.logwarn("[collision_checker] Detected a collision!!!")

        check_duration = (rospy.Time.now() - time_before_check).to_sec()
        rospy.logwarn("[collision_checker] Collision check took %.3f s", check_duration)
        return

    def PointcloudCallback(self, pointcloud_msg):
        pointcloud_array = convert_pointcloud2_to_array(pointcloud_msg)
        # Filter out the points corresponding to all drones poses
        self.CheckForCollision(pointcloud_array, pointcloud_msg.header.frame_id)

    def OdometryCallback(self, odometry_msg):
        # Get the linear speed of the drone
        self._current_pose = odometry_msg.pose.pose
        self._current_Twist = odometry_msg.twist.twist

if __name__ == '__main__':
    try:
        CollisionChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
