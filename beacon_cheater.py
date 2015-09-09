#!/usr/bin/env python

import rospy
import sys
from assignment1.srv import *
from beacon_finder import SERVICE_BEACON_CHEATER

def start():
    rospy.init_node('beacon_cheater')
    if len(sys.argv) < 4:
        print 'give me topColor, bottomColor, angle'
        return
    top = sys.argv[1]
    bottom = sys.argv[2]
    angle = float(sys.argv[3])
    rospy.wait_for_service(SERVICE_BEACON_CHEATER)
    cheat = rospy.ServiceProxy(SERVICE_BEACON_CHEATER, BeaconCheat)
    status = cheat(top, bottom, angle)
    print 'Cheated: ', status


if __name__ == '__main__':
    start()