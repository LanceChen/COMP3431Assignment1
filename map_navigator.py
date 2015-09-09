#!/usr/bin/env python
# coding=utf-8

"""
A map navigator service that accepts a goal position and then automatically controls the robot to move to the goal point
 along with the computed shortest path.
"""
SERVICE_START_NAVIGATION = 'start_navigation'
SERVICE_GENERATE_NAVIGATION_TARGET = 'generate_navigation_target'

__author__ = 'kelvin'

import math
from threading import RLock

import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from assignment1.msg import *

from assignment1.srv import *

from search_path import a_star_search
from search_path import preprocess_map
from search_path import optimize_path
from search_path import convert_point_to_map_cell
from search_path import convert_way_point_to_map_cell
from search_path import convert_map_cell_to_point
from search_path import safety_margin_percentage
from constants import robot_radius, beacon_radius

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
current_goal_distance = None
nav_target_list = None
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
goal_lock = RLock()


def set_goal(goal, angle, goal_distance):
    global current_goal, current_goal_angle, current_goal_distance
    if goal is None:
        current_goal = None
        current_goal_angle = None
        current_goal_distance = None
        rospy.loginfo('Navigation goal has been cleared')
    else:
        current_goal = goal
        current_goal_angle = angle
        current_goal_distance = goal_distance
        if angle is not None and (angle < -math.pi or angle > math.pi):
            current_goal_angle = 0.0
            rospy.logwarn('Bad angle parameter: %f, reset to %f' % (angle, current_goal_angle))
        rospy.loginfo('New navigation goal has been set: (%d, %d) angle=%s goal_distance=%d'
                      % (goal.x, goal.y, angle, goal_distance))


def handle_start_navigation(request):
    """
    @type request: StartNavigationRequest
    """
    global nav_target_list
    with goal_lock:
        rospy.loginfo('New navigation list has arrived (length=%d)' % len(request.targets))
        nav_target_list = request.targets
        if len(request.targets) <= 0:
            set_goal(None, None, None)
        else:
            # since we cannot use None in Service call, we use a flag to indicate that angles are omitted
            for index, target in enumerate(request.targets):
                print '=== Target ', (index + 1), ' ==='
                print target
            first_target = nav_target_list.pop(0)
            angle = first_target.angle if not first_target.ignore_angle else None
            set_goal(first_target.point, angle, first_target.goal_distance)
        return StartNavigationResponse()


def handle_generate_nav_target(request):
    """
    @type request: GenerateNavigationTargetRequest
    """
    tf_listener.waitForTransform(map_frame, robot_frame, rospy.Time(), rospy.Duration(5))
    t = tf_listener.getLatestCommonTime(map_frame, robot_frame)
    (transform, quaternion) = tf_listener.lookupTransform(map_frame, robot_frame, t)
    robot_angle = euler_from_quaternion(quaternion)[2]
    target_angle = robot_angle + request.angle
    target_x = transform[0] + math.cos(target_angle) * request.range
    target_y = transform[1] + math.sin(target_angle) * request.range
    resolution = current_map.info.resolution
    point = (int(math.floor(target_x / resolution)), int(math.floor(target_y / resolution)))
    response = GenerateNavigationTargetResponse()
    target = NavigationTarget()
    target.point = WayPoint(point[0], point[1])
    target.goal_distance = int(
        math.ceil((beacon_radius + robot_radius * (1 + safety_margin_percentage)) / resolution)) + 1
    target.ignore_angle = True
    response.target = target
    return response


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


def start_navigator():
    global tf_listener, nav_pub, last_map_augmented_occ, last_path, last_goal, last_map, \
        last_target_point, last_robot_point, is_watch_clockwise, last_path_fail, is_watch_mode, last_watch_angle, \
        current_goal_angle
    rospy.init_node('map_navigator')
    tf_listener = tf.TransformListener()
    rospy.Service(SERVICE_START_NAVIGATION, StartNavigation, handle_start_navigation)
    rospy.Service(SERVICE_GENERATE_NAVIGATION_TARGET, GenerateNavigationTarget, handle_generate_nav_target)
    # noinspection PyTypeChecker
    rospy.Subscriber(map_topic, OccupancyGrid, map_received)
    nav_pub = rospy.Publisher(robot_control_topic, Twist, queue_size=1)
    rospy.loginfo('Map Navigator node started')
    rate = rospy.Rate(working_frequency)
    while not rospy.is_shutdown():
        if current_goal is not None and current_map is not None:
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
                if last_map != current_map or last_goal != current_goal:
                    print 'Running a*...'
                    current_goal_cell = convert_way_point_to_map_cell(current_map, current_goal)
                    robot_point_cell = convert_point_to_map_cell(current_map, robot_point)
                    last_path = a_star_search(current_map, last_map_augmented_occ, robot_point_cell, current_goal_cell,
                                              current_goal_distance)
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
                    else:
                        if current_goal_angle is None and current_goal_distance > 0:
                            current_goal_angle = math.atan2(current_goal.y - robot_point[1],
                                                            current_goal.x - robot_point[0])
                        if current_goal_angle is None or adjust_angle(robot_angle, current_goal_angle):
                            if len(nav_target_list) > 0:
                                next_nav_target = nav_target_list.pop(0)
                                rospy.loginfo('Navigation to (%d, %d) (angle=%f) has been finished!' %
                                              (current_goal.x, current_goal.y, current_goal_angle))
                                angle = next_nav_target.angle if not next_nav_target.ignore_angle else None
                                set_goal(next_nav_target.point, angle, next_nav_target.goal_distance)
                            else:
                                rospy.loginfo('Navigation of ALL targets has been finished!')
                                set_goal(None, None, None)
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
