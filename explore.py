#!/usr/bin/env python

 

import rospy
import tf
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

navi_pub = None
mode = 1
MinDist = 0.3
threshold = 0.1
FindWall = 1
Align = 2
FollowWall = 3
Adjust = 4

# 0.5 angular = 14 seconds revolution

def callback(data):
    global mode

    if mode == FindWall:
        rospy.loginfo("findWall")
        findWall(data)
    elif mode == Align:
        rospy.loginfo("align")
        align(data)
    elif mode == FollowWall:
        rospy.loginfo("followWall")
        followWall(data)
    elif mode == Adjust:
        rospy.loginfo("adjust")
        adjust(data)


def findWall(data):
    global mode

    # check if we're within range of wall
    for i in range(len(data.ranges)):
        if data.ranges[i] < MinDist:
            mode = Align

    # else go forward
    if mode == FindWall:
        moveCmd = Twist()
        moveCmd.linear.x = 0.1
        moveCmd.angular.z = 0
        navi_pub.publish(moveCmd)


def align(data):
    global mode
    global threshold

    mid = len(data.ranges) / 2
    offset = 256
    thetaInd = mid + offset
    threshold = 0.1

    # if we are aligned then follow the wall, else rotate
    if (data.ranges[thetaInd] < MinDist + threshold and
        data.ranges[thetaInd] > MinDist - threshold):
        mode = FollowWall
    else:
        moveCmd = Twist()
        moveCmd.linear.x = 0
        moveCmd.angular.z = 0.5
        navi_pub.publish(moveCmd)


def followWall(data):
    global mode
    global threshold

    mid = len(data.ranges) / 2
    offset = 256
    thetaInd = mid + offset

    # if within wall range then keep moving forward
    # else adjust
    if (data.ranges[thetaInd] < MinDist + threshold and
        data.ranges[thetaInd] > MinDist - threshold):
        moveCmd = Twist()
        moveCmd.linear.x = 0.1
        moveCmd.angular.z = 0
        navi_pub.publish(moveCmd)
    else:
        mode = Adjust


def adjust(data):
    global mode
    global threshold

    mid = len(data.ranges) / 2
    offset = 256
    thetaInd = mid + offset

    # if not within range then turn robot until in range
    if (data.ranges[thetaInd] > MinDist + threshold):
        moveCmd = Twist()
        moveCmd.linear.x = 0
        moveCmd.angular.z = 0.5
        navi_pub.publish(moveCmd)
    else:
        mode = FollowWall


# http://answers.ros.org/question/31815/getting-coordinates-of-turtlebot/
def getCurPos():
    # set up
    pBase = PoseStamped()
    pMap = PoseStamped()
    listener = tf.TransformListener()

    # get pose of robot transformed into frame of map
    pBase.header.frame_id = "base_link"
    pBase.pose.position.x = 0
    pBase.pose.position.y = 0
    pBase.pose.orientation = tf().createQuaternionMsgFromYaw(0)
    current_transform = rospy().now()
    listener.getLatestCommonTime(pBase.header.frame_id, "map", current_transform)
    pBase.header.stamp = current_transform
    listener.transformPose("map", pBase, pMap)

    return pMap


def main():
    global navi_pub
    rospy.init_node('explorer', anonymous=True)
    navi_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, callback)
    r = rospy.Rate(10)
    rospy.spin()


if __name__ == '__main__':
    main()