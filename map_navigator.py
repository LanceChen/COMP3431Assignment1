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

from assignment1.msg import *

from search_path import a_star_search
from search_path import preprocess_map

robot_frame = '/base_link'
robot_control_topic = '/cmd_vel_mux/input/navi'
map_topic = '/map'
map_frame = '/map'
working_frequency = 10  # hz
angle_close_threshold = math.radians(3)
speed_angular_base = 0.2
speed_angular_haste = 1.0
speed_linear_base = 0.2
speed_linear_log_haste = 0.5

tf_listener = None
nav_pub = None
current_goal = None
current_map = None
last_goal = None
last_map = None
last_map_augmented_occ = None
last_path = None
last_target_point = None
last_robot_point = None
goal_lock = RLock()


def set_goal(goal):
    global current_goal
    with goal_lock:
        if goal is None:
            current_goal = None
            rospy.loginfo('Navigation goal has been cleared')
        else:
            current_goal = (goal.x, goal.y)  # convert back to normal sequence
            rospy.loginfo('New navigation goal has been set: (%d, %d)' % (goal.x, goal.y))


def handle_start_navigation(request):
    set_goal(request.goal)
    return StartNavigationResponse()


def map_received(map_grid):
    global current_map
    current_map = map_grid


def move_robot(linear, angular):
    action = Twist()
    action.linear.x = linear
    action.angular.z = angular
    nav_pub.publish(action)


def preprocess_path(path):
    new_path = []
    last_point = None
    last_angle = None
    is_start_point = True
    for way_point in path:
        point = (way_point.x, way_point.y)  # convert back to normal sequence
        if is_start_point:
            is_start_point = False
        else:
            current_angle = math.atan2(point[1] - last_point[1], point[0] - last_point[0])
            if last_angle is not None and math.fabs(last_angle - current_angle) < angle_close_threshold:
                new_path.pop()
            else:
                last_angle = current_angle
        last_point = point
        new_path.append(point)
    return new_path


def start_navigator():
    global tf_listener, nav_pub, last_map_augmented_occ, last_path, last_goal, last_map, \
        last_target_point, last_robot_point
    rospy.init_node('map_navigator', anonymous=True)
    tf_listener = tf.TransformListener()
    rospy.Service('start_navigation', StartNavigation, handle_start_navigation)
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
                robot_point = (int(transform[0] / resolution), int(transform[1] / resolution))
                if last_robot_point is not None and last_robot_point != robot_point:
                    print "Move from %s to %s" % (str(last_robot_point), str(robot_point))
                last_robot_point = robot_point
                if last_map != current_map:
                    print 'Preprocessing map...'
                    last_map_augmented_occ = preprocess_map(current_map)
                if last_map != current_map or last_goal != current_goal:
                    print 'Running a*...'
                    last_path = a_star_search(current_map, WayPoint(robot_point[0], robot_point[1]),
                                              WayPoint(current_goal[0], current_goal[1]),
                                              augmented_occ=last_map_augmented_occ)
                    print 'Returned path length: %d' % len(last_path)
                    if len(last_path) == 0:
                        rospy.logwarn('Failed to find a path from (%d, %d) to (%d, %d)' %
                                      (robot_point[0], robot_point[1], current_goal[0], current_goal[1]))
                    else:
                        last_path = preprocess_path(last_path)
                        print 'Optimized path length: %d' % len(last_path)
                        last_target_point = last_path.pop()  # pop first target point (start point)
                        print 'Start point %s' % str(last_target_point)
                last_goal = current_goal
                last_map = current_map
                while robot_point == last_target_point:
                    if len(last_path) > 0:
                        last_target_point = last_path.pop()  # pop next target point
                        print 'Next target point %s' % str(last_target_point)
                        print 'Point remains: %d' % len(last_path)
                    else:
                        last_target_point = None
                        break
                if last_target_point is None:
                    rospy.loginfo('Navigation finished!')
                    set_goal(None)
                    move_robot(0, 0)
                else:
                    dx = (last_target_point[0] + 0.5) * resolution - transform[0]
                    dy = (last_target_point[1] + 0.5) * resolution - transform[1]
                    dist = math.sqrt(dx * dx + dy * dy)
                    robot_angle = euler_from_quaternion(quaternion)[2]
                    target_angle = math.atan2(dy, dx)
                    delta_angle = robot_angle - target_angle
                    if delta_angle > math.pi:
                        delta_angle -= math.pi * 2
                    elif delta_angle < -math.pi:
                        delta_angle += math.pi * 2
                    if delta_angle < -angle_close_threshold:
                        move_robot(0, speed_angular_base + (-delta_angle) / math.pi * speed_angular_haste)
                    elif delta_angle > angle_close_threshold:
                        move_robot(0, -speed_angular_base - delta_angle / math.pi * speed_angular_haste)
                    else:
                        move_robot(speed_linear_base + math.log(1 + dist, 2) * speed_linear_log_haste, 0)
        rate.sleep()


if __name__ == '__main__':
    try:
        start_navigator()
    except rospy.ROSInterruptException:
        pass
