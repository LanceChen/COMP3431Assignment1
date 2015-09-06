#!/usr/bin/env python
# coding=utf-8

"""
A map navigator service that accepts a goal position and then automatically controls the robot to move to the goal point
 along with the computed shortest path.
"""

__author__ = 'kelvin'

import math
from threading import RLock

import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

from assignment1.srv import *
from constants import *

from search_path import a_star_search
from search_path import preprocess_map
from search_path import optimize_path
from search_path import convert_point_to_map_cell
from search_path import convert_way_point_to_map_cell
from search_path import convert_map_cell_to_point
from search_path import get_max_map_size_limit

robot_frame = '/base_link'
robot_control_topic = '/cmd_vel_mux/input/navi'
map_topic = '/map'
map_frame = '/map'
working_frequency = 10  # hz
angle_close_threshold = math.radians(2)
angle_watch_area = math.radians(90)
speed_angular_base = 0.2
speed_angular_haste = 2.0
speed_angular_watch = 0.4
speed_linear_base = 0.2
speed_linear_log_haste = 0.1

tf_listener = None
nav_pub = None
current_goal = None
current_goal_angle = None
current_map = None
last_goal = None
last_map = None
last_map_augmented_occ = None
last_path = None
last_target_point = None
last_robot_point = None
last_path_fail = False
last_watch_angle = 0.0
is_watch_mode = False
is_watch_clockwise = True
is_explore_mode = False
need_explore_next = True
map_size_limit = None
visited_checkpoints = set()
goal_lock = RLock()


def set_goal(goal, angle, explore_mode=False):
    global current_goal, current_goal_angle, is_explore_mode, need_explore_next
    with goal_lock:
        is_explore_mode = explore_mode
        if explore_mode:
            current_goal = None
            current_goal_angle = None
            need_explore_next = True
            rospy.loginfo('Exploration mode has been activated')
        elif goal is None:
            current_goal = None
            current_goal_angle = None
            rospy.loginfo('Navigation goal has been cleared')
        else:
            current_goal = goal
            current_goal_angle = angle
            if angle is not None and (angle < -math.pi or angle > math.pi):
                current_goal_angle = 0.0
                rospy.logwarn('Bad angle parameter: %f, reset to %f' % (angle, current_goal_angle))
            rospy.loginfo('New navigation goal has been set: (%d, %d) angle=%s' % (goal.x, goal.y, angle))


def handle_start_navigation(request):
    if request.stop:
        set_goal(None, None)
    elif request.explore_mode:
        set_goal(None, None, True)
    else:
        angle = request.angle if not request.ignore_angle else None
        set_goal(request.goal, angle)
    return StartNavigationResponse()


def map_received(map_grid):
    global current_map
    with goal_lock:
        current_map = map_grid


def move_robot(linear, angular):
    action = Twist()
    action.linear.x = linear
    action.angular.z = angular
    nav_pub.publish(action)


def get_angle_diff(angle_a, angle_b):
    """Get the normalized angle difference between two normalized angles.
    Normalized angles should be in the range of (-pi, pi]
    """
    delta_angle = angle_a - angle_b
    if delta_angle > math.pi:
        delta_angle -= math.pi * 2
    elif delta_angle <= -math.pi:
        delta_angle += math.pi * 2
    return delta_angle


def adjust_angle(robot_angle, target_angle):
    """Adjust the robot's angle and return True when adjustment has been finished"""
    delta_angle = get_angle_diff(robot_angle, target_angle)
    if delta_angle < -angle_close_threshold:
        move_robot(0, speed_angular_base + (-delta_angle) / math.pi * speed_angular_haste)
    elif delta_angle > angle_close_threshold:
        move_robot(0, -speed_angular_base - delta_angle / math.pi * speed_angular_haste)
    else:
        return True
    return False

def compute_map_size_limit(map_grid):
    width = map_grid.info.width
    height = map_grid.info.height
    min_i = height
    max_i = -1
    min_j = width
    max_j = -1
    for i in range(height):
        for j in range(width):
            val = map_grid.data[i * width + j]
            if val == -1:
                continue
            if min_i > i:
                min_i = i
            if max_i < i:
                max_i = i
            if min_j > j:
                min_j = j
            if max_j < j:
                max_j = j
    if min_i == height:
        rospy.logwarn('Tried to compute the size limit of an empty map')
        return 0, width-1, 0, height - 1
    return min_j, max_j, min_i, max_i


def explore_next():
    global map_size_limit
    robot_point_cell = convert_point_to_map_cell(current_map, last_robot_point)
    width = current_map.info.width
    height = current_map.info.height
    resolution = current_map.info.resolution
    min_y, max_y, min_x, max_x = get_max_map_size_limit(current_map)
    point_not_reachable = (min(max_x, width-1), min(max_y, height-1))
    if map_size_limit is None:  # when we don't know the outer boundary of the map
        # check if we know that we cannot reach a point that is in fact not reachable, which means if we've got the
        # boundary shape of the map
        if len(a_star_search(current_map, last_map_augmented_occ, robot_point_cell, point_not_reachable)) == 0:
            map_size_limit = compute_map_size_limit(current_map)
            print 'Map size limit confirmed: %s - %s' % (
                str(convert_map_cell_to_point(current_map, (map_size_limit[0], map_size_limit[2]))),
                str(convert_map_cell_to_point(current_map, (map_size_limit[1], map_size_limit[3])))
            )
    checkpoint_size = int(math.ceil(robot_radius / resolution))
    robot_x = int(round(1.0 * robot_point_cell[0] / checkpoint_size))
    robot_y = int(round(1.0 * robot_point_cell[1] / checkpoint_size))
    if map_size_limit is not None:
        min_x, max_x, min_y, max_y = map_size_limit[0], map_size_limit[1], map_size_limit[2], map_size_limit[3]
    points_to_check = 4  # find 4 most close checkpoints that need to visit and finally pick the nearest one
    n = 1
    nearest_checkpoint_cell = None
    nearest_checkpoint_path = None
    nearest_checkpoint_path_length = float('inf')
    while points_to_check > 0:
        for m in range(-n+1, n+1):  # [-n+1, n+1)
            all_out_of_bound = True
            for px, py in [(robot_x+n, robot_y+m), (robot_x-n, robot_y-m),
                           (robot_x-m, robot_y+n), (robot_x+m, robot_y-n)]:
                cell = (px*checkpoint_size, py*checkpoint_size)
                if min_x <= cell[0] <= max_x and min_y <= cell[1] <= max_y:
                    all_out_of_bound = False
                    if (px, py) not in visited_checkpoints:
                        point = convert_map_cell_to_point(current_map, cell)
                        print 'Running a* towards ', str(point)
                        path = a_star_search(current_map, last_map_augmented_occ, robot_point_cell, cell)
                        path_length = len(path)
                        print 'Returned path length: %d' % path_length
                        if path_length == 0:
                            print 'Failed to find a path from (%d, %d) to (%d, %d)' % \
                                  (last_robot_point[0], last_robot_point[1], point[0], point[1])
                            visited_checkpoints.add((px, py))
                        else:
                            points_to_check -= 1
                            if float(path_length) < nearest_checkpoint_path_length:
                                nearest_checkpoint_cell = cell
                                nearest_checkpoint_path = path
                                nearest_checkpoint_path_length = path_length

            if all_out_of_bound:
                break
        n += 1
    return nearest_checkpoint_cell, nearest_checkpoint_path


def start_navigator():
    global tf_listener, nav_pub, last_map_augmented_occ, last_path, last_goal, last_map, \
        last_target_point, last_robot_point, is_watch_clockwise, last_path_fail, is_watch_mode, last_watch_angle
    rospy.init_node('map_navigator')
    tf_listener = tf.TransformListener()
    rospy.Service('start_navigation', StartNavigation, handle_start_navigation)
    # noinspection PyTypeChecker
    rospy.Subscriber(map_topic, OccupancyGrid, map_received)
    nav_pub = rospy.Publisher(robot_control_topic, Twist, queue_size=1)
    rospy.loginfo('Map Navigator node started')
    rate = rospy.Rate(working_frequency)
    while not rospy.is_shutdown():
        if (current_goal is not None or is_explore_mode) and current_map is not None:
            with goal_lock:
                try:
                    t = tf_listener.getLatestCommonTime(map_frame, robot_frame)
                    (transform, quaternion) = tf_listener.lookupTransform(map_frame, robot_frame, t)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                    rospy.logerr('Failed to lookup transform: %s' % e)
                    move_robot(0, 0)
                    rate.sleep()
                    continue
                resolution = current_map.info.resolution
                robot_point = (int(math.floor(transform[0] / resolution)), int(math.floor(transform[1] / resolution)))
                if last_robot_point is not None and last_robot_point != robot_point:
                    print "Move from %s to %s" % (str(last_robot_point), str(robot_point))
                last_robot_point = robot_point
                if last_map != current_map:
                    print 'Preprocessing map...'
                    last_map_augmented_occ = preprocess_map(current_map)
                if is_explore_mode:
                    if need_explore_next:
                        last_path_fail = False
                        cell, path = explore_next()
                        if cell is None:
                            rospy.loginfo('No more points to explore')
                        else:
                            print 'Next checkpoint: ', convert_map_cell_to_point(current_map, cell)
                            print 'Optimizing path...'
                            last_path = optimize_path(current_map, last_map_augmented_occ, path)
                            print 'Optimized path length: %d' % len(last_path)
                            last_target_point = convert_map_cell_to_point(current_map, last_path.pop())
                            print 'Start point %s' % str(last_target_point)
                elif last_map != current_map or last_goal != current_goal:
                    print 'Running a*...'
                    current_goal_cell = convert_way_point_to_map_cell(current_map, current_goal)
                    robot_point_cell = convert_point_to_map_cell(current_map, robot_point)
                    last_path = a_star_search(current_map, last_map_augmented_occ, robot_point_cell, current_goal_cell)
                    print 'Returned path length: %d' % len(last_path)
                    if len(last_path) > 0:
                        print 'Optimizing path...'
                        last_path = optimize_path(current_map, last_map_augmented_occ, last_path)
                        print 'Optimized path length: %d' % len(last_path)
                    if len(last_path) == 0:
                        rospy.logwarn('Failed to find a path from (%d, %d) to (%d, %d)' %
                                      (robot_point[0], robot_point[1], current_goal.x, current_goal.y))
                        last_target_point = None
                        last_path_fail = True
                    else:
                        # pop first target point (start point)
                        last_target_point = convert_map_cell_to_point(current_map, last_path.pop())
                        print 'Start point %s' % str(last_target_point)
                        last_path_fail = False
                last_goal = current_goal
                last_map = current_map
                if last_path_fail:
                    move_robot(0, angle_watch_area)  # stay here and watch around
                    rate.sleep()
                    continue
                # keep while structure in case we want to change equality check to similarity check
                while robot_point == last_target_point:
                    if len(last_path) > 0:
                        # pop next target point
                        next_cell = last_path.pop()
                        if next_cell[0] == -1:
                            is_watch_mode = True
                            last_target_point = None
                            last_watch_angle = next_cell[1]
                            print 'Unknown area ahead, wait and watch %f degree' % math.degrees(last_watch_angle)
                        else:
                            is_watch_mode = False
                            last_target_point = convert_map_cell_to_point(current_map, next_cell)
                            print 'Next target point %s' % str(last_target_point)
                        print 'Point remains: %d' % len(last_path)
                    else:
                        last_target_point = None
                        break
                robot_angle = euler_from_quaternion(quaternion)[2]
                if last_target_point is None:
                    if is_watch_mode:
                        delta_angle = get_angle_diff(robot_angle, last_watch_angle)
                        if delta_angle < -angle_watch_area / 2:
                            is_watch_clockwise = False
                        elif delta_angle > angle_watch_area / 2:
                            is_watch_clockwise = True
                        if is_watch_clockwise:
                            move_robot(0, -speed_angular_watch)
                        else:
                            move_robot(0, speed_angular_watch)
                    elif current_goal_angle is None or adjust_angle(robot_angle, current_goal_angle):
                        rospy.loginfo('Navigation finished!')
                        set_goal(None, None)
                        move_robot(0, 0)
                else:  # move to target point
                    dx = (last_target_point[0] + 0.5) * resolution - transform[0]
                    dy = (last_target_point[1] + 0.5) * resolution - transform[1]
                    dist = math.sqrt(dx * dx + dy * dy)
                    target_angle = math.atan2(dy, dx)
                    if adjust_angle(robot_angle, target_angle):
                        move_robot(speed_linear_base + math.log(1 + dist, 2) * speed_linear_log_haste, 0)
        rate.sleep()


if __name__ == '__main__':
    try:
        start_navigator()
    except rospy.ROSInterruptException:
        pass
