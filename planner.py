#!usr/bin/python

import roslib
import sys
import rospy
from beacons import BeaconObject
from assignment1.msg import *
from assignment1.srv import *


class planner:
    def _init_(self):
        self.beaconsToSearch = []
        self.foundBeacons = []
        self.visitedBeacons = []
        self.beaconSubscriber = rospy.Subscriber("beacons found", BeaconList, self.callback)
        rospy.init_node('planner', anonymous=True)

    def callback(self):
        for foundb in BeaconList.foundBeacons:
            newBeacon = BeaconObject()
            newBeacon.top = foundb.top
            newBeacon.bottom = foundb.bottom
            newBeacon.x = foundb.x
            newBeacon.y = foundb.y
            if self.inBeaconsToSearch(newBeacon):
                self.foundBeacons.append(newBeacon)

    def addSearchBeacon(self, bcObj):
        self.beaconsToSearch.append(bcObj)

    def addFoundBeacon(self, bcObj):
        self.foundBeacons.append(bcObj)

    def addVisitedBeacon(self, bcObj):
        self.visitedBeacons.append(bcObj)

    def allBeaconsFound(self):
        if len(self.beaconsToSearch) == len(self.foundBeacons):
            return True
        else:
            return False

    def allBeaconsVisisted(self):
        if len(self.beaconsToSearch) == len(self.visitedBeacons):
            return True
        else:
            return False

    def inBeaconsToSearch(self, bcObj):
        result = False
        for b in self.beaconsToSearch:
            if b.top == bcObj.top and b.bottom == bcObj.bottom:
                result = True
                break
        return result

if __name__ == '__main__':
    pl = planner()

    for b in sys.argv:
        colours = b.split(str=",", num=2)
        newBeacon = BeaconObject()
        newBeacon.top = colours[0]
        newBeacon.bottom = colours[1]
        pl.addSearchBeacon(newBeacon)

    while not rospy.is_shutdown():
        if not pl.allBeaconsFound():
            #explore
            explore
        elif not pl.allBeaconsVisisted():
            #navigate to way points
            visit