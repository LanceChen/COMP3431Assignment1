#!/usr/bin/env python


import rospy
import tf
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

navi_pub = None
MinDist = 0.4
MaxDist = 0.8
prev = None
Close = 0
Far = 1
Other = 2

# 0.5 angular = 14 seconds revolution

def move(x, z):
    moveCmd = Twist()
    moveCmd.linear.x = x
    moveCmd.angular.z = z
    navi_pub.publish(moveCmd)


def getThetaInd(data):
    mid = len(data.ranges) / 2
    offset = 256
    thetaInd = mid + offset
    return thetaInd


def tooClose(data):
    close = False
    thetaInd = getThetaInd(data)

    if data.ranges[thetaInd] < MinDist:
        close = True

    return close


def tooFar(data):
    far = False
    thetaInd = getThetaInd(data)

    if data.ranges[thetaInd] > MaxDist:
        far = True

    return far


def wallAhead(data):
    ahead = False
    mid = len(data.ranges) / 2

    if data.ranges[mid] < MinDist:
        ahead = True

    return ahead


def callback(data):
    global prev

    # if robot too close to the wall then turn left
    # if the robot is too far from the wall turn right
    # otherwise go straight
    # also if we are at corner
    # (switching between tooClose and tooFar quickly)
    # then just move forward for short time to get out of infinite loop
    if (wallAhead(data)):
        move(0, -0.5) # turn right
        prev = Other
    elif (tooClose(data)):
        if prev == Far:
            move(0.1, 0) # just go forward
            #time.sleep(1)
        else:
            move(0, -0.5) # turn right
        prev = Close
    elif (tooFar(data)):
        if prev == Close:
            move(0.1, 0) # just go forward
            #time.sleep(1)
        else:
            move(0, 0.5) # turn right
        prev = Far
    else:
        move(0.1, 0) # just go forward
        prev = Other


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
