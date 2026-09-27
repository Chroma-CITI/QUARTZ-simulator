#!/usr/bin/env python3
# coding: utf-8

import rospy
from random import shuffle
from geometry_msgs.msg import Pose, Point, Quaternion

def main():
    # Initialize the ROS node
    rospy.init_node('target_publisher', anonymous=True)

    # Get world-related information from parameters
    shoud_shuffle = rospy.get_param('~shuffle_poses', False)
    world_name = rospy.get_param('~world_name', 'not_specified')
    rospy.logwarn("Defining a trajectory for the world: %s", world_name)

    # Create a publisher
    target_pub = rospy.Publisher("topic_target", Pose, queue_size=10)

    # Example poses
    # WARNING: these targets are used to compute a speed by comparing them to the odom of the robot.
    # The odom is expressed in world frame so the targets are expected to be expressed in the world
    # frame. This is bad practice though... 

    example_poses = list()
    if world_name == "kitchen":
        example_poses = [
            Pose(position=Point(2.6, 5.2, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(position=Point(2.7, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(position=Point(-1.8, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(position=Point(-1.8, -3.7, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(position=Point(1.8, -5.4, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        ]
    elif world_name == "city":
        example_poses = [
            Pose(position=Point(17.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(position=Point(-11.0, 5.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(position=Point(-14.0, 26.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(position=Point(6.0, 65.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(position=Point(44.0, 22.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        ]
    elif world_name == "gaz_station":
        example_poses = [
            Pose(position=Point(-1.26, 5.77, 1.0), orientation=Quaternion(0.0, 0.0, 0.962, 0.271)),
            Pose(position=Point(-14.68, 6.914, 1.0), orientation=Quaternion(0.0, 0.0, -0.90, 0.42)),
            Pose(position=Point(-14.68, -12.578, 1.0), orientation=Quaternion(0.0, 0.0, -0.742, 0.67)),
            Pose(position=Point(0.0, -12.578, 1.0), orientation=Quaternion(0.0, 0.0, 0.504, 0.86)),
            Pose(position=Point(0.0, 5.77, 1.0), orientation=Quaternion(0.0, 0.0, 0.504, 0.86)),
        ]
    else:
        example_poses = [
                Pose(position=Point(1.0, 2.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                Pose(position=Point(-3.0, 2.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                Pose(position=Point(3.0, -3.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            ]
    # Shuffle so that if more than one drone is following the same traj,
    # they do not necessarly go to the same points at the same time.
    if shoud_shuffle:
        shuffle(example_poses)

    # Publish the pose array at a fixed rate
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        for pose in example_poses:
            target_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
