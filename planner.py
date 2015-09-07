#!usr/bin/python

import rospy
from assignment1.msg import *
from assignment1.srv import *

from explore import SERVICE_STOP_EXPLORE
from map_navigator import SERVICE_START_NAVIGATION
from image_recognition import SERVICE_STOP_MONITOR_CAMERA
from image_recognition import TOPIC_BEACONS_IN_CAMERA


class Planner:
    def __init__(self):
        self.beaconsToSearch = []
        self.foundBeacons = []
        rospy.init_node('planner', anonymous=True)
        # noinspection PyTypeChecker
        self.beaconSubscriber = rospy.Subscriber(TOPIC_BEACONS_IN_CAMERA, BeaconList, self.callback)
        # wait and initialize proxies for all related services
        rospy.wait_for_service(SERVICE_STOP_EXPLORE)
        rospy.wait_for_service(SERVICE_STOP_MONITOR_CAMERA)
        rospy.wait_for_service(SERVICE_START_NAVIGATION)
        self.stop_explore = rospy.ServiceProxy(SERVICE_STOP_EXPLORE, StopExplore)
        self.stop_monitor_camera = rospy.ServiceProxy(SERVICE_STOP_MONITOR_CAMERA, StopMonitorCamera)
        self.start_navigation = rospy.ServiceProxy(SERVICE_START_NAVIGATION, StartNavigation)

    def callback(self, beacon_list):
        """
        @type beacon_list: BeaconList
        """
        for found_bc in beacon_list.foundBeacons:
            # TODO: add logic to avoid append duplicates
            if self.in_beacons_to_search(found_bc):
                self.foundBeacons.append(found_bc)

                # TODO check if all beacons found, and call the services here

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


if __name__ == '__main__':
    pl = Planner()
    # TODO use rospy.get_param to retrieve target beacons, see file '.../comp3431/assign1/launch/demo.launch'
    rospy.get_param('beacons')
    rospy.spin()
