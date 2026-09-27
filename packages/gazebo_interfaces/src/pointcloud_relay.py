#!/usr/bin/env python3
# coding: utf-8

import rospy
import tf2_ros

import numpy as np

from sensor_msgs.msg import PointCloud2

from pointcloud_helper import convert_pointcloud2_to_array, convert_array_to_pointcloud2, get_neighbors_indices

class PointCloudRelay:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pointcloud_relay', anonymous=True)

        # Get parameters
        self._debug = rospy.get_param('/debug', False)
        # Expecting topics as follow. /droneName_NUMBER/TopicName
        # FIXME: this architecture forces the param /drone_name to be already loaded to the rosparams
        droneName = rospy.get_param('/drone_name')
        self._neighbors_radius = rospy.get_param('/robot_radius', 0.5)
        self._topicNamespace = droneName + "_"
        self._topicName = rospy.get_param('~topicName')
        self._numberOfTopics = rospy.get_param('~numberOfTopics')

        # Subscriber for PointCloud2 messages
        self._subscribers = list()
        # Should work for N number of topics
        for topicNumber in range(self._numberOfTopics):
            fullTopic = "/" + self._topicNamespace + str(topicNumber) + '/' + self._topicName
            rospy.logdebug("Subscribing to topic %s", fullTopic)
            self._subscribers.append(
                rospy.Subscriber(fullTopic, PointCloud2, self.PointcloudCallback)
            )

        # Create a publisher
        self.target_pub = rospy.Publisher("all_pointclouds", PointCloud2, queue_size=10)
        if self._debug:
            self.debug_pub = rospy.Publisher("removed_pointclouds", PointCloud2, queue_size=10)

        # Initialize the TF2 buffer and listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def FilterOutDrones(self, pointcloud_array, pointcloud_frame_id):
        # Create a mask from which the drone points will be filtered out
        mask = np.ones(len(pointcloud_array), dtype=bool)
        # The number of topics should correspond to the number of drones
        for drone_index in range(self._numberOfTopics):
            # Get the drone pose in the current frame
            try:
                drone_frame_id = self._topicNamespace + str(drone_index) + "/base_link" # Expecting frame id as follow. droneName_NUMBER/base_link
                # Wait for the transform from the trajectoryFrameId to the _localFrame to be available
                rospy.logdebug("[pointcloud_relay] Listening to the transform between frame %s and frame %s", pointcloud_frame_id, drone_frame_id)
                dronePoseInCurrentPointcloudFrame = self._tf_buffer.lookup_transform(
                    pointcloud_frame_id,  # Target frame: the frame in which the pose will be transform to
                    drone_frame_id,  # Source frame: the frame in which the pose will be transform from
                    rospy.Time(0),  # Get the latest available transform
                    rospy.Duration(1.0)  # Timeout duration in s
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Transform failed: {e}")
            drone_pose = np.array(
                [
                    dronePoseInCurrentPointcloudFrame.transform.translation.x,
                    dronePoseInCurrentPointcloudFrame.transform.translation.y,
                    dronePoseInCurrentPointcloudFrame.transform.translation.z
                ]
            )
            # Get the indices of the drone's neighbor points
            neighbors_indices = get_neighbors_indices(pointcloud_array, drone_pose, self._neighbors_radius)
            rospy.logdebug("[pointcloud_relay] Removing %i neigbors for drone %s", len(neighbors_indices), drone_frame_id)

            # Mask all neighbors points
            mask[neighbors_indices] = False

            #Publish debug pointcloud
            if self._debug:
                debug_pointcloud = pointcloud_array[~mask]
                debug_pointcloud_out = convert_array_to_pointcloud2(debug_pointcloud, rospy.Time.now(), pointcloud_frame_id)
                self.debug_pub.publish(debug_pointcloud_out)

        return pointcloud_array[mask]

    def PointcloudCallback(self, pointcloudMsg):
        pointcloud_array = convert_pointcloud2_to_array(pointcloudMsg)
        # Filter out the points corresponding to all drones poses
        filtered_pointcloud_array = self.FilterOutDrones(pointcloud_array, pointcloudMsg.header.frame_id)
        # Convert the filtered pointcloud back to a ros pointcloud2 msg
        pointcloud_out = convert_array_to_pointcloud2(filtered_pointcloud_array, rospy.Time.now(), pointcloudMsg.header.frame_id)
        # Relaying input topics' messages to the output topic
        self.target_pub.publish(pointcloud_out)

if __name__ == '__main__':
    try:
        PointCloudRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
