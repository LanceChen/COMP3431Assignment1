#!/usr/bin/env python
# coding=utf-8

"""An interactive testing tool for validating A* algorithm in Search Path Service.

This script reads in given map file, and shows the map in GUI. The user can 'right-click' any reasonable point to
specify it as the starting point or the goal point (in this order), and then it will call A* algorithm to compute the
shortest path, which will be rendered using blue arrows in the map. The start/end point will be filled with purple.

The user can also 'left-click' to switch to different map modes:
    1. Original map: show the occupancy grid using red-based colors. At each block, the bigger its occ value is, the
        darker the color will be (occ=0: white, occ=100: black). One exception is the invalid block, which will be
        filled with green color.
    2. Closed-set map: show the final state of the closed set from the A* algorithm on top of the original map. Each
        block within the closed set will be filled with light blue.
    3. Open-set map: show the final state of the open set from the A* algorithm on top of the original map. Each block
        within the open set will be filled with orange.
"""

__author__ = 'kelvin'

import sys
import time
from colorsys import hls_to_rgb
import Tkinter

from assignment1.msg import WayPoint
from nav_msgs.msg import OccupancyGrid

from search_path import a_star_search

unit = 20  # pixel per block in the map
map_mode = 0  # map mode: [0]normal [1]closed set [2]open set
canvas = None  # Tkinter canvas for rendering the map
start_point = None  # start point of the requested path
goal_point = None  # goal point of the requested path
grid = None  # the occupancy grid (original map data)
closed_set = set()  # closed set of A* algorithm
open_set = set()  # open set of A* algorithm
came_from = {}  # "came from" dict of A* algorithm
path = []  # shortest path computed by A* algorithm


def start():
    """Start testing tool"""
    global unit, start_point, goal_point, grid, closed_set, open_set, came_from, path, canvas
    # handle param
    if len(sys.argv) < 2:
        print 'Give me map file'
        exit()
    input_map = sys.argv[1]
    if len(sys.argv) > 2:
        unit = int(sys.argv[2])
    read_map(input_map)

    # init map
    top = Tkinter.Tk()
    canvas = Tkinter.Canvas(top, bg="green", height=grid.info.height * unit, width=grid.info.width * unit)
    draw_map()
    canvas.bind("<Button-1>", change_mode)
    canvas.bind("<Button-3>", change_points)
    canvas.pack()
    top.mainloop()


# noinspection PyUnusedLocal
def change_mode(event):
    """change map mode"""
    global map_mode
    map_mode = (map_mode + 1) % 3
    draw_map()


def change_points(event):
    """change start/goal point"""
    global start_point, goal_point, path, closed_set, open_set, came_from
    x = event.x / unit
    y = event.y / unit
    if start_point is None:
        start_point = WayPoint(x, y)
    elif goal_point is None:
        goal_point = WayPoint(x, y)
        call_a_star()
    else:
        path = []
        closed_set = set()
        open_set = set()
        came_from = {}
        start_point = WayPoint(x, y)
        goal_point = None
    draw_map()


def call_a_star():
    """call A* algorithm"""
    global path, closed_set, open_set, came_from
    time_start = time.clock()
    path = a_star_search(grid, start_point, goal_point, closed_set=closed_set, open_set=open_set,
                         came_from=came_from)
    time_elapsed = time.clock() - time_start
    if len(path) <= 0:
        sys.stderr.write("Warning: A* returns no result!\n")
    else:
        print 'A* finished in %ss, path length = %d' % (str(time_elapsed), len(path))


def read_map(map_file):
    """Read map from given text file"""
    global grid
    grid = OccupancyGrid()
    grid.info.resolution = 1
    grid.data = list()
    first_line = True
    with open(map_file, 'r') as f:
        for line in f:
            if first_line:
                param = [s for s in line.split(' ') if s]
                grid.info.width = int(param[0])
                grid.info.height = int(param[1])
                first_line = False
            else:
                for num in line.strip('\n').split(' '):
                    if num:
                        grid.data.append(int(num))
    assert len(grid.data) == grid.info.width * grid.info.height


def draw_map():
    """Render the map using canvas"""
    canvas.delete('all')
    width = grid.info.width
    height = grid.info.height
    for row in range(height):
        for col in range(width):
            val = grid.data[row * width + col]
            color = 'green'
            if val != -1:
                rgb = hls_to_rgb(0, (100 - val) / 100.0, 1)
                color = '#%0.2X%0.2X%0.2X' % (int(255 * rgb[0]), int(255 * rgb[1]), int(255 * rgb[2]))
            canvas.create_rectangle(col * unit, row * unit, (col + 1) * unit, (row + 1) * unit, fill=color)
    if map_mode == 1:
        for point in closed_set:
            canvas.create_rectangle(point[0] * unit, point[1] * unit, (point[0] + 1) * unit, (point[1] + 1) * unit,
                                    fill='lightblue')
    if map_mode == 2:
        for point in open_set:
            canvas.create_rectangle(point[0] * unit, point[1] * unit, (point[0] + 1) * unit, (point[1] + 1) * unit,
                                    fill='orange')
    if start_point is not None:
        canvas.create_rectangle(start_point.x * unit, start_point.y * unit, (start_point.x + 1) * unit,
                                (start_point.y + 1) * unit, fill='purple')
    if goal_point is not None:
        canvas.create_rectangle(goal_point.x * unit, goal_point.y * unit, (goal_point.x + 1) * unit,
                                (goal_point.y + 1) * unit, fill='purple')
    directions = [
        ['↖', '↑', '↗'],
        ['←', 'x', '→'],
        ['↙', '↓', '↘']
    ]
    for point in [(p.x, p.y) for p in path]:
        if point not in came_from:
            continue
        source_point = came_from[point]
        x = point[0] - source_point[0] + 1
        y = point[1] - source_point[1] + 1
        canvas.create_text((source_point[0] + 0.5) * unit, (source_point[1] + 0.5) * unit, text=directions[y][x],
                           font=("Ubuntu", unit),
                           fill='blue')


if __name__ == '__main__':
    start()
