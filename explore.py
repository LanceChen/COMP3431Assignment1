#!/usr/bin/env python

import math

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from assignment1.srv import *

SERVICE_GET_RANGE_FROM_ANGLE = 'get_range_from_angle'
SERVICE_STOP_EXPLORE = 'stop_explore'

NaviPub = None
ForwardVel = 0.2
RightVel = -0.5
LeftVel = 0.5
MinDist = 0.4
MaxDist = 0.8
Prev = None
Close = 0
Far = 1
Other = 2
Done = False
latest_laser_scan = None


def handle_range_query(request):
    """Callback function of '/get_range_from_angle' service.
    It returns the distance/range for the given angle based on the latest laser scan data
    @type request: GetRangeFromAngleRequest
    """
    response = GetRangeFromAngleResponse()
    response.range = angle_to_dist(request.angle, latest_laser_scan)
    return response


# handle the callback of the explore node
def callback(data):
    global Prev, latest_laser_scan
    latest_laser_scan = data  # store the latest laser info for range queries
    if Done:
        return

    # if the wall is ahead then turn right
    # else if we are too close to the wall turn right (accounting for infinite loop bug)
    # else if we are too far from the wall turn left (accounting for infinite loop bug)
    # else we'll just go forward
#    if almost_hit(data):
#        move(-ForwardVel, RightVel)
#        Prev = Other
    if wall_ahead(data):
        move(0, RightVel)
        Prev = Other
    elif too_close(data):
        # if going into infinite loop between 2 states then go forward to get out of it
        # else turn right
        if Prev == Far:
            move(ForwardVel, 0)
        elif Prev == Other:
            move(ForwardVel, 0)
        else:
            move(0, RightVel)
        Prev = Close
    elif too_far(data):
        # if going into infinite loop between 2 states then go forward to get out of it
        # else turn left
        if Prev == Close:
            move(ForwardVel, 0)
        elif Prev == Other:
            move(ForwardVel, 0)
        else:
            move(0, LeftVel)
        Prev = Far
    else:
        if almost_hit_right(data):
            move(-ForwardVel, 0.2)
            Prev = Other
        elif almost_hit_left(data):
            move(-ForwardVel, -0.2)
            Prev = Other
        else:
            move(ForwardVel, 0)
            Prev = Other


# checks if about to hit something on right
def almost_hit_right(data):
    almost = False

    last = len(data.ranges)
    mid = last / 2

    for i in range(0, mid):
        if data.ranges[i] < MinDist / 2.5:
            almost = True
            break

    return almost


# checks if about to hit something on left
def almost_hit_left(data):
    almost = False

    last = len(data.ranges)
    mid = last / 2

    for i in range(mid, last):
        if data.ranges[i] < MinDist / 2.5:
            almost = True
            break

    return almost


# handles the service that stops the turtlebot
# noinspection PyUnusedLocal
def handle_stop_explore(request):
    global Done
    rospy.loginfo('Explore node stopped')
    Done = True
    return StopExploreResponse()


# determines if there is a wall straight ahead of the turtlebot
def wall_ahead(data):
    ahead = False
    mid = len(data.ranges) / 2

    if data.ranges[mid] < MinDist:
        ahead = True

    return ahead


# determines if the turtlebot is too close to the wall
def too_close(data):
    close = False
    theta_ind = get_theta_ind(data)

    if data.ranges[theta_ind] < MinDist:
        close = True

    return close


# determines if the turtlebot is too far from the wall
def too_far(data):
    far = False
    theta_ind = get_theta_ind(data)

    if data.ranges[theta_ind] > MaxDist:
        far = True

    return far


# general function to move the robot
def move(x, z):
    move_cmd = Twist()
    move_cmd.linear.x = x
    move_cmd.angular.z = z
    NaviPub.publish(move_cmd)


# get the index for the angle theta used for checking where the wall is
def get_theta_ind(data):
    mid = len(data.ranges) / 2
    offset = 256
    theta_ind = mid + offset
    return theta_ind


# converts from degrees to radians
def deg_to_rad(degrees):
    rad = degrees * math.pi / 180
    return rad


# converts a data range index to an angle in radians
def int_to_angle(index, data):
    min_angle = -(3 * math.pi) / 4
    angle = min_angle + index * data.angle_increment
    return angle


# converts an angle in radians to a data range index
def angle_to_ind(theta, data):
    min_angle = -(3 * math.pi) / 4
    angle_dif = theta - min_angle
    ind = int(round(angle_dif / data.angle_increment))
    return ind


# gets the distance of the closest object at an angle in radians
def angle_to_dist(angle, data):
    ind = angle_to_ind(angle, data)
    dist = data.ranges[ind]
    return dist


def main():
    global NaviPub
    rospy.init_node('explorer', anonymous=True)
    rospy.Service(SERVICE_STOP_EXPLORE, StopExplore, handle_stop_explore)
    rospy.Service(SERVICE_GET_RANGE_FROM_ANGLE, GetRangeFromAngle, handle_range_query)
    NaviPub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    # noinspection PyTypeChecker
    rospy.Subscriber("/scan", LaserScan, callback)
    # TODO: check why failed to "run" when restart this script
    rospy.loginfo('Explore node started')
    rospy.spin()


if __name__ == '__main__':
    main()
