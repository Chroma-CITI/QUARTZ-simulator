#!/usr/bin/env python3
# coding: utf-8

import time
import rospy
import tf2_ros
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2
from pointcloud_helper import convert_pointcloud2_to_array, convert_array_to_pointcloud2


def create_fov_pointcloud(vertical_fov, horizontal_fov, min_range, max_range, angular_resolution=0.035, resolution=1.0):
    # Generate vertical and horizontal angles
    vertical_angles = np.arange(vertical_fov[0], vertical_fov[1], angular_resolution)
    horizontal_angles = np.arange(horizontal_fov[0], horizontal_fov[1], angular_resolution)

    # Create a grid of angles
    v_grid, h_grid = np.meshgrid(vertical_angles, horizontal_angles, indexing='ij')

    # Calculate the Cartesian coordinates for the FOV points
    ranges = np.linspace(min_range, max_range, int((max_range - min_range) / resolution))
    x = np.outer(ranges, np.cos(v_grid) * np.sin(h_grid)).ravel()
    y = np.outer(ranges, np.cos(v_grid) * np.cos(h_grid)).ravel()
    z = np.outer(ranges, np.sin(v_grid)).ravel()

    # Stack the coordinates to create the FOV point cloud
    fov_pointcloud = np.column_stack((x, y, z))

    return fov_pointcloud

def remove_occluded_points(fov_pointcloud, pointcloud):
    # Normalize the FOV point cloud
    fov_norms = np.linalg.norm(fov_pointcloud, axis=1, keepdims=True)
    fov_directions = fov_pointcloud / fov_norms

    # Normalize the second point cloud
    point_norms = np.linalg.norm(pointcloud, axis=1, keepdims=True)
    point_directions = pointcloud / point_norms

    # Compute the dot products between each point direction and FOV direction
    dot_products = np.dot(fov_directions, point_directions.T)

    # Create a mask that identifies occluded points
    # fov_norms > point_norms is expanded to match the shape of dot_products
    occluded_mask = (dot_products > 0.999) & (fov_norms > point_norms.T)

    # Remove the occluded points
    # We need to select the indices of FOV points that are not occluded by any of the points in pointcloud
    keep_mask = np.all(occluded_mask == False, axis=1)
    fov_pointcloud = fov_pointcloud[keep_mask]

    return fov_pointcloud

def remove_occluded_points_cp(fov_pointcloud, pointcloud):
    import cupy as cp # NOTE: is it costly? normally python check if it is already imported
    # Convert input arrays to CuPy arrays
    cp_fov_pointcloud = cp.asarray(fov_pointcloud)
    cp_pointcloud = cp.asarray(pointcloud)

    # Normalize the FOV point cloud
    fov_norms = cp.linalg.norm(cp_fov_pointcloud, axis=1, keepdims=True)
    fov_directions = cp_fov_pointcloud / fov_norms

    # Normalize the second point cloud
    point_norms = cp.linalg.norm(cp_pointcloud, axis=1, keepdims=True)
    point_directions = cp_pointcloud / point_norms

    # Compute the dot products between each point direction and FOV direction
    # dot_products = np.dot(fov_directions, point_directions.T)
    # dot_products = (csr_matrix(fov_directions) @ csr_matrix(point_directions.T)).toarray()
    dot_products = cp.dot(fov_directions, point_directions.T)

    # Create a mask that identifies occluded points
    # fov_norms > point_norms is expanded to match the shape of dot_products
    occluded_mask = (dot_products > 0.999) & (fov_norms > point_norms.T)

    # Remove the occluded points
    # We need to select the indices of FOV points that are not occluded by any of the points in pointcloud
    keep_mask = cp.all(occluded_mask == False, axis=1)
    cp_fov_pointcloud = cp_fov_pointcloud[keep_mask]

    # Convert the result back to a NumPy array if needed
    cp_fov_pointcloud = cp.asnumpy(cp_fov_pointcloud)

    # convert back the result to numpy array
    filtered_out_numpy_array = cp.asnumpy(cp_fov_pointcloud)

    # Clean up GPU memory
    del cp_fov_pointcloud, cp_pointcloud, fov_norms, point_norms, dot_products
    cp.get_default_memory_pool().free_all_blocks()
    cp.cuda.Device().synchronize()

    return filtered_out_numpy_array

class ComplementaryPointCloudComputer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('complementary_pointcloud_computer', anonymous=True)
        self._rate = rospy.Rate(30)

        self._using_cupy = rospy.get_param('~using_cupy', False)

        self._pointcloud_sub = rospy.Subscriber("pointcloud_in", PointCloud2, self.PointcloudCallback, queue_size=1)
        self.complementary_pointcloud_pub = rospy.Publisher("pointcloud_out", PointCloud2, queue_size=1)

        # Initialize the TF2 buffer and listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._sensor_origin = np.array([0.0, 0.0, 0.0], dtype='float32')

        # Sensor parameters
        self.max_input_age_in_sec_ = rospy.get_param("~max_input_age_in_sec", 0.12)  # In seconds
        self.max_processing_time_in_sec_ = rospy.get_param("~max_processing_time_in_sec", 0.12)  # In seconds
        # NOTE: self._vertical_fovs and resolutions have been changed for speedup
        self._min_range = rospy.get_param("/lidar_min_range", 0.1)  # Minimum range in meters
        self._max_range = rospy.get_param("/lidar_max_range", 10.0)  # Maximum range in meters
        self._global_resolution =  0.5 # One point every resolution_in_m. Should be equivalent to voxel_size
        vertical_min_angle = rospy.get_param("/vertical_min_angle", -14.0)  # Min angle in rad
        vertical_max_angle = rospy.get_param("/vertical_max_angle", 50.0)  # max angle in rad
        self._vertical_fovs = [np.radians(vertical_min_angle), np.radians(vertical_max_angle)]  # Vertical Field of View in radians
        vertical_fov = self._vertical_fovs[0] + self._vertical_fovs[1] # TODO: modulo 3.14
        self._horizontal_fovs = [-np.radians(180), np.radians(180)]  # Horizontal Field of View in radians
        horizontal_fov = self._horizontal_fovs[0] + self._horizontal_fovs[1] # TODO: modulo 3.14
        self._vertical_resolution = max(0.08, np.radians(vertical_fov) / 100.0)  # Vertical resolution in radians
        self._horizontal_resolution = max(0.1, np.radians(horizontal_fov) / 100.0)  # Horizontal resolution in radians

    def PointcloudCallback(self, pointcloud_msg):
        # check if the msg is not too old
        msg_age = (rospy.Time.now() - pointcloud_msg.header.stamp).to_sec()
        if msg_age > self.max_input_age_in_sec_:
            rospy.logwarn("Dropped pointcloud: too old (%.3f s)", msg_age)
            return

        pointcloud_array = convert_pointcloud2_to_array(pointcloud_msg)

        start_time = time.time()
        fov_pointcloud = create_fov_pointcloud(self._vertical_fovs, self._horizontal_fovs, self._min_range, self._max_range, self._horizontal_resolution, self._global_resolution)
        end_time = time.time()
        duration = end_time - start_time
        # print(f"create_fov_pointcloud execution time: {duration:.4f} seconds")

        start_time = time.time()
        if self._using_cupy:
            new_freepointcloud_array = remove_occluded_points_cp(fov_pointcloud, pointcloud_array)
        else:
            new_freepointcloud_array = remove_occluded_points(fov_pointcloud, pointcloud_array)
        end_time = time.time()
        duration = end_time - start_time
        print(f"remove_occluded_points execution time: {duration:.4f} seconds")

        pointcloud_out = convert_array_to_pointcloud2(new_freepointcloud_array, rospy.Time.now(), pointcloud_msg.header.frame_id)

        time_difference_between_pointcloud_and_complementary_pointcloud = (rospy.Time.now() - pointcloud_msg.header.stamp).to_sec()
        if time_difference_between_pointcloud_and_complementary_pointcloud > self.max_processing_time_in_sec_:
            rospy.logwarn("Processed pointcloud too old to publish (%.3f s delay)", time_difference_between_pointcloud_and_complementary_pointcloud)
            return
        
        self.complementary_pointcloud_pub.publish(pointcloud_out)

    def Spin(self):
        while(not rospy.is_shutdown()):
            self._rate.sleep()

if __name__ == '__main__':
    try:
        ComplementaryPointCloudComputer().Spin()
    except rospy.ROSInterruptException:
        pass
