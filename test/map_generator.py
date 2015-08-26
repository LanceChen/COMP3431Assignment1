#!/usr/bin/env python
# coding=utf-8

"""A map file generator for testing Search Path Service

It's a simple subscriber node that listen to the "/map" topic. When new map data comes, it will clip the map data
(remove redundant -1 values), and save it into the specified file with simple text format. Then you may use
"search_path_tester.py" to test the Search Path Service.
"""

__author__ = 'kelvin'

import sys

import rospy
from nav_msgs.msg import OccupancyGrid

output_file = None


def map_received(map_grid):
    """Callback function for receiving map data"""
    width = map_grid.info.width
    height = map_grid.info.height
    rospy.loginfo('Received map data: width=%d, height=%d, resolution=%f' %
                  (width, height, map_grid.info.resolution))
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
        print 'Empty map'
        return
    if min_i > 0:
        min_i -= 1
    if max_i < height - 1:
        max_i += 1
    if min_j > 0:
        min_j -= 1
    if max_j < width - 1:
        max_j += 1
    clip_width = max_j - min_j + 1
    clip_height = max_i - min_i + 1
    print 'Clipped size: %d * %d: ' % (clip_width, clip_height)
    with open(output_file, 'w') as f:
        f.write("%d %d\n" % (clip_width, clip_height))
        for i in range(min_i, max_i + 1):
            for j in range(min_j, max_j + 1):
                f.write("%d " % map_grid.data[i * width + j])
            f.write('\n')


def start():
    """Start subscribe node"""
    global output_file
    if len(sys.argv) < 2:
        print 'Tell me where to store the map file.'
        exit()
    output_file = sys.argv[1]
    rospy.init_node('map_generator', anonymous=True)
    # noinspection PyTypeChecker
    rospy.Subscriber('/map', OccupancyGrid, map_received)
    rospy.loginfo('Map generator started.')
    rospy.spin()


if __name__ == '__main__':
    start()
