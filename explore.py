__author__ = 'adam'

import rospy
import search_path
from nav_msgs.msg import OccupancyGrid

done = False
beaconsFound = 0

def callback(map):
    global done
    global beaconsFound

    if done == False:
        nextCell = findNextCell(map)
        path = findPath(nextCell)
        traverse()

        if foundBeacon():
            beaconsFound += 1
            if beaconsFound == 4:
                done = True


def findNextCell(map):
    # convert current position into xy

    # make array of -1 positions and their xy positions

    unexplored = []

    for 

    # find closest xy position



    return 0


def findPath(nextCell):
    rospy.wait_for_service('search_path')
    try:
        return SearchPath()
    except rospy.ServiceException, e:
        print "Service call failed"


def foundBeacon():
    # call beaconRecognition service
    return False

def traverse():
    return False


def main():
    rospy.init_node('explorer', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    rospy.spin()


if __name__ == '__main__':
    main()