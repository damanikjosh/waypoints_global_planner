#!/usr/bin/env python3
import rospkg
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


from path_planning.Bezier import Bezier
from path_planning.BezierSpline import BezierSpline
from path_planning.CubicSpline import CubicSpline
# from path_planning.Dubins import Dubins
from path_planning.Line import Line
from path_planning.Ellipse import Ellipse

DUBINS_POINTS = [[0,0],[3,5]]
POINTS = [[0,0],[1,-2],[10.5,-4.5],[5,6],[10,15],[25,30]]
ELLIPSE_POINTS = [[0,0],[1,2],[3,1]]
CIRCLE_POINTS  = [[4,4],[6,9],[9,6]]
ORIENTATION = [[180, 90]]
LINE_POINTS = [[0,0],[7,-12]]

INIT_POSITION = [-2.25, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position


def path_coord_to_gazebo_coord(x, y):
    RADIUS = 0.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)


rospack = rospkg.RosPack()
jackal_helper = rospack.get_path('jackal_helper')

desired_path = Path()
path_array = np.load(f'{jackal_helper}/worlds/BARN/path_files/path_{world_idx}.npy')
path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
path_array = np.array(path_array) - np.array(INIT_POSITION[:2])
path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
path_array = np.insert(path_array, len(path_array),
                       (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)


def prepare_desired_path(trajectory_type):
    '''
    Prepare desired path
    '''
    # if trajectory_type == "dubins":
    #     shape = Dubins(DUBINS_POINTS,ORIENTATION,5)
    
    if trajectory_type == "circle":
        shape = Ellipse(CIRCLE_POINTS)

    if trajectory_type == "bezier":
        shape = Bezier(POINTS, False)

    if trajectory_type == "ellipse":
        shape = Ellipse(ELLIPSE_POINTS)

    if trajectory_type == "cubic_spline":
        shape = CubicSpline(POINTS)

    if trajectory_type == "bezier_spline":
        shape = BezierSpline(POINTS, order=3, is_periodic=False)

    if trajectory_type == "line":
        shape = Line(LINE_POINTS, is_periodic=False)

    samples, yaw_samples = shape.sample_points()
    return samples, yaw_samples

'''
if __name__ == '__main__':
    path = prepare_desired_path("circle")
    print path
'''