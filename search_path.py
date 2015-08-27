#!/usr/bin/env python
# coding=utf-8

"""Search Path Service for ROS based on A* algorithm.

This service provides A* algorithm to compute the shortest path between two points in the OccupancyGrid map.
"""

__author__ = 'kelvin'

import math
import heapq

import rospy
from nav_msgs.msg import OccupancyGrid
from assignment1.msg import *
from assignment1.srv import *

occ_threshold = 60
inf = float('inf')


def euclidean_distance(x1, y1, x2, y2):
    """Get the euclidean distance between point (x1, y1) and (x2, y2)."""
    dx = (x1 - x2)
    dy = (y1 - y2)
    return math.sqrt(dx * dx + dy * dy)


def map_distance(map_grid, start, end):
    """Get the map distance between start point and end point.

    If occupancy >= occ_threshold, it will return Infinity, otherwise return the euclidean distance.
    @type map_grid: OccupancyGrid
    @type start: (int, int)
    @type end: (int, int)
    """
    w = map_grid.info.width
    goal_occ = map_grid.data[end[1] * w + end[0]]
    if goal_occ >= occ_threshold:
        return inf
    return euclidean_distance(start[0], start[1], end[0], end[1])


def cost_estimate(start, end):
    """Estimate the cost from start point to end point.

    Here we just simply use euclidean distance.
    @type start: (int, int)
    @type end: (int, int)
    """
    return euclidean_distance(start[0], start[1], end[0], end[1])


def reconstruct_path(came_from, current):
    """Reconstruct the WayPoint path from the initial(start) point to the current point.

    @type came_from: dict
    @type current: (int, int)
    """
    total_path = [WayPoint(current[0], current[1])]
    while current in came_from:
        current = came_from[current]
        total_path.append(WayPoint(current[0], current[1]))
    return total_path


def is_valid_point(map_grid, point):
    """Check if a point in the map is valid.
    A point is invalid if and only if the point is out of the range of the map or it's occ value is -1.
    @type map_grid: OccupancyGrid
    @type point: (int, int)
    """
    x = point[0]
    y = point[1]
    width = map_grid.info.width
    height = map_grid.info.height
    return 0 <= x < width and 0 <= y < height and map_grid.data[y * width + x] != -1


def neighbour_points(map_grid, point):
    """Get the valid neighbouring points of a given point based on the map info.
    @type map_grid: OccupancyGrid
    @type point: (int, int)
    """
    point_x = point[0]
    point_y = point[1]
    neighbours = [
        (point_x - 1, point_y - 1), (point_x, point_y - 1), (point_x + 1, point_y - 1),
        (point_x - 1, point_y), (point_x, point_y), (point_x + 1, point_y),
        (point_x - 1, point_y + 1), (point_x, point_y + 1), (point_x + 1, point_y + 1)
    ]
    return [p for p in neighbours if is_valid_point(map_grid, p)]


def a_star_search(map_grid, start, goal, **kwargs):
    """Use A* algorithm to compute the shortest path from the start point to the goal point in the given map.
    @type map_grid: OccupancyGrid
    @type start: WayPoint
    @type goal: WayPoint
    """
    closed_set = kwargs['closed_set'] if 'closed_set' in kwargs else set()
    open_set = kwargs['open_set'] if 'open_set' in kwargs else set()
    came_from = kwargs['came_from'] if 'came_from' in kwargs else dict()
    g_score = kwargs['g_score'] if 'g_score' in kwargs else dict()
    f_score = kwargs['f_score'] if 'f_score' in kwargs else dict()
    open_heap = []  # priority queue for fast retrieval of the open point with the smallest f_score

    # convert back to simple sequence
    start_seq = (start.x, start.y)
    goal_seq = (goal.x, goal.y)
    g_score[start_seq] = 0
    f_score[start_seq] = cost_estimate(start_seq, goal_seq)
    open_set.add(start_seq)
    heapq.heappush(open_heap, (f_score[start_seq], start_seq))

    # when open set is not empty
    while len(open_heap) > 0:
        # pop the point with the smallest f_score from the open set
        current_f_score, current_point = heapq.heappop(open_heap)
        open_set.remove(current_point)
        # if already reach the goal
        if current_point == goal_seq:
            return reconstruct_path(came_from, goal_seq)
        # add current point into closed set (do not compute for it any more)
        closed_set.add(current_point)
        # get all valid neighbouring points of current point
        for neighbour_point in neighbour_points(map_grid, current_point):
            # if already closed
            if neighbour_point in closed_set:
                continue
            # compute tentative g_score
            tentative_g_score = g_score.get(current_point, inf) + map_distance(map_grid, current_point, neighbour_point)
            # if already open
            in_open_set = neighbour_point in open_set
            if not in_open_set or tentative_g_score < g_score.get(neighbour_point, inf):
                # update came_from and scores
                came_from[neighbour_point] = current_point
                g_score[neighbour_point] = tentative_g_score
                f_score[neighbour_point] = tentative_g_score + cost_estimate(neighbour_point, goal_seq)
                if not in_open_set:
                    # push it into open set
                    heapq.heappush(open_heap, (f_score[neighbour_point], neighbour_point))
                    open_set.add(neighbour_point)
    return []


def handle_search_path(request):
    """Handler function for a single service call.
    @type request: SearchPathRequest
    """
    map_info = request.map.info
    rospy.loginfo('Searching shortest path from (%d, %d) to (%d, %d) on a %d * %d map...' %
                  (request.start.x, request.start.y, request.goal.x, request.goal.y,
                   map_info.width, map_info.height))
    path = a_star_search(request.map, request.start, request.goal)
    if len(path) <= 0:
        rospy.logwarn('Search path failed')
    else:
        rospy.loginfo('Found shortest path, length=%d' % len(path))
        for i in range(len(path)):
            point = path[i]
            print '(%d, %d)->' % (point.x, point.y)
    return SearchPathResponse(path)


def search_path_server():
    """Initialize "search_path_server" node and register "search_path" service."""
    rospy.init_node('search_path_server')
    rospy.Service('search_path', SearchPath, handle_search_path)
    rospy.loginfo('Search path server is ready.')
    rospy.spin()


if __name__ == '__main__':
    search_path_server()
