#!/usr/bin/env python

import rospy
from assignment1.msg import *
from assignment1.srv import *

from explore import SERVICE_STOP_EXPLORE
from map_navigator import SERVICE_START_NAVIGATION
from beacon_finder import SERVICE_STOP_MONITOR_CAMERA
from beacon_finder import TOPIC_BEACONS_IN_CAMERA

stop_explore = None
stop_monitor_camera = None
start_navigation = None

class Planner:
    def __init__(self):
        global stop_explore, stop_monitor_camera, start_navigation
        self.beaconsToSearch = []
        self.foundBeacons = []
        rospy.init_node('planner', anonymous=True)
        # noinspection PyTypeChecker
        self.beaconSubscriber = rospy.Subscriber(TOPIC_BEACONS_IN_CAMERA, BeaconList, self.callback)
        # wait and initialize proxies for all related services
        rospy.wait_for_service(SERVICE_STOP_EXPLORE)
        rospy.wait_for_service(SERVICE_STOP_MONITOR_CAMERA)
        rospy.wait_for_service(SERVICE_START_NAVIGATION)
        stop_explore = rospy.ServiceProxy(SERVICE_STOP_EXPLORE, StopExplore)
        stop_monitor_camera = rospy.ServiceProxy(SERVICE_STOP_MONITOR_CAMERA, StopMonitorCamera)
        start_navigation = rospy.ServiceProxy(SERVICE_START_NAVIGATION, StartNavigation)
        rospy.loginfo('Planner node started')

    def callback(self, beacon_list):
        """
        @type beacon_list: BeaconList
        """
        for found_bc in beacon_list.foundBeacons:

            if self.in_beacons_to_search(found_bc) and not self.in_found_beacons(found_bc):
                self.foundBeacons.append(found_bc)
                rospy.loginfo("Beacon found: %s" % (str(found_bc)))

        if self.all_beacons_found():
            stop_explore()
            stop_monitor_camera()
            nav_target_list = []
            for visit_bc in self.beaconsToSearch:
                nav_target = visit_bc.target
                nav_target_list.append(nav_target)
            start_navigation(nav_target_list, False)

    def all_beacons_found(self):
        if len(self.beaconsToSearch) == len(self.foundBeacons):
            return True
        else:
            return False

    def in_beacons_to_search(self, bc):
        result = False
        for bc_search in self.beaconsToSearch:
            if bc_search.topColour == bc.topColour and bc_search.bottomColour == bc.bottomColour:
                result = True
                break
        return result

    def in_found_beacons(self, bc):
        result = False
        for bc_found in self.foundBeacons:
            if bc_found.topColour == bc.topColour and bc_found.bottomColour == bc.bottomColour:
                result = True
                break
        return result


if __name__ == '__main__':
    pl = Planner()
    beacons = rospy.get_param('beacons')
    str_beacons = ''
    for beacon in beacons:
        new_beacon = Beacon()
        new_beacon.topColour = beacons[beacon]['top']
        new_beacon.bottomColour = beacons[beacon]['bottom']
        pl.beaconsToSearch.append(new_beacon)
        str_beacons += '{top:%s,bottom:%s} ' % (new_beacon.topColour, new_beacon.bottomColour)
    rospy.loginfo("Beacons to search: %s" % str_beacons)
    rospy.spin()
