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
        self.beaconSubscriber = rospy.Subscriber("beacons found", BeaconList, callback)
        rospy.init_node('listener', anonymous=True)

    def callback(self):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        foundBeacons =  BeaconList.foundBeacons

    def addSearchBeacon(self, BeaconObject):
        self.beaconsToSearch.append(BeaconObject)

    def allBeaconsFound(self):
        result = False
        for search in self.beaconsToSearch:
            for found in self.foundBeacons:
                if search.top == found.top and search.bottom == found.bottom:
                    result =True
                    break
            if not result:
                break
        return result

    def allBeaconsVisisted(self):
        result = False
        for search in self.beaconsToSearch:
            for visited in self.visitedBeacons:
                if search.top == visited.top and search.bottom == visited.bottom:
                    result =True
                    break
            if not result:
                break
        return result

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

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
        elif not pl.allBeaconsVisisted():
            #navigate to way points