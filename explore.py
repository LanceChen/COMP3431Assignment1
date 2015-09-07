#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from assignment1.srv import *

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


# handle the callback of the explore node
def callback(data):
    global Prev

    # if the wall is ahead then turn right
    # else if we are too close to the wall turn right (accounting for infinite loop bug)
    # else if we are too far from the wall turn left (accounting for infinite loop bug)
    # else we'll just go forward
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
        move(ForwardVel, 0)
        Prev = Other


# handles the service that stops the turtlebot
def handle_stop_explore(request):
    global Done
    rospy.loginfo('Stopped')
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
    ind = angle_dif / data.angle_increment
    return ind


# gets the distance of the closest object at an angle in radians
def angle_to_dist(angle, data):
    ind = angle_to_ind(angle, data)
    dist = data.ranges[ind]
    return dist


def main():
    global NaviPub
    rospy.init_node('explorer', anonymous=True)
    rospy.Service('stop_explore', StopExplore, handle_stop_explore)
    NaviPub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    # noinspection PyTypeChecker
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.Rate(10)
    rospy.spin()


if __name__ == '__main__':
    main()