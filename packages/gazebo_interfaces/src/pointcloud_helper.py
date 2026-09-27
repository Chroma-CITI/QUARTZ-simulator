#!/usr/bin/python3

import ros_numpy
import numpy as np
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2

def convert_pointcloud2_to_array(pointcloud_msg):
    # HACK: intelaero does not have this issue, to be investigated...
    if len(pointcloud_msg.fields) == 4:
        pointcloud_msg.fields[3].name = 'distance' # HACK: sounds wrong, checking gazebo_plugin__velodyn.h
    # pointcloud_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud_msg, remove_nans=True)
    pointcloud_np = ros_numpy.numpify(pointcloud_msg).reshape(pointcloud_msg.width, pointcloud_msg.height)

    return ros_numpy.point_cloud2.get_xyz_points(pointcloud_np)

def get_neighbors_indices(xyz_points, query_point, neighbors_radius):
    """Given a pointcloud and a point in it, find all the query point's neighbors in a given radius

    Args:
        xyz_points (np.array): xyz.shape -> (N, 3) and xyz[i] = [x_i, y_i, z_i]
        query_point (np.array): [x0, y0, z0], around which the neighbors are searched
        neighbors_radius (float): Same unit as xyz_points and query_point

    Returns:
        _type_: _description_
    """
    tree = cKDTree(xyz_points)
    return tree.query_ball_point(query_point, r=neighbors_radius)


def select_points_in_sub_sphere(xyz_points, sub_sphere_radius):
    # Calculate the squared distances from the origin (0, 0, 0)
    distances_squared = np.sum(xyz_points**2, axis=1)
    
    # Compare the squared distances to the squared radius
    return distances_squared <= sub_sphere_radius**2

def convert_array_to_pointcloud2(array_np, stamp, frame_id):
    dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
    pointcloud = np.zeros(array_np.shape[0], dtype=dtype)
    pointcloud['x'] = array_np[:, 0]
    pointcloud['y'] = array_np[:, 1]
    pointcloud['z'] = array_np[:, 2]

    msg = ros_numpy.msgify(PointCloud2, pointcloud)
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    return msg