#!/usr/bin/env python3
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from path_planning.Bezier import Bezier
import path_planning.PoseHelper as PoseHelper

world_idx = 0
id = "odom"

INIT_POSITION = [-2.25, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position


def path_coord_to_gazebo_coord(x, y):
    RADIUS = 0.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)


def prepare_path():
    rospack = rospkg.RosPack()
    jackal_helper = rospack.get_path('jackal_helper')

    desired_path = Path()
    path_array = np.load(f'{jackal_helper}/worlds/BARN/path_files/path_{world_idx}.npy')
    # path_array = path_array[10:]
    path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
    path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
    path_array = np.insert(path_array, len(path_array),
                           (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
    path_array = np.array(path_array) - np.array(INIT_POSITION[:2])

    shape = Bezier(path_array, False)
    #
    samples, yaw_samples = shape.sample_points()
    # samples = path_array


    for t, point in enumerate(samples, start=0):
        pose = PoseHelper.prepare_posestamped(samples[t][0], samples[t][1], 1.57)
        pose.header.seq = t 
        pose.header.frame_id = id
        pose.header.stamp = rospy.get_rostime()
        desired_path.poses.append(pose)

    desired_path.header.frame_id = id
    desired_path.header.stamp = rospy.get_rostime()
    desired_path_pub.publish(desired_path) 
    desired_path = []

def path_publisher():
    global desired_path_pub
    desired_path_pub = rospy.Publisher('/desired_path', Path, queue_size=1)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        prepare_path()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('desired_path_node')
    rospy.loginfo("desired_path node is started!!")
    world_idx =  rospy.get_param('~world_idx')
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass
