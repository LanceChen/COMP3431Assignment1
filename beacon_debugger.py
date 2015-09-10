#!/usr/bin/env python

import rospy
import sys
from assignment1.srv import *
from beacon_finder import SERVICE_BEACON_DEBUGGER

def start():
    rospy.init_node('beacon_debugger')
    if len(sys.argv) < 4:
        print 'give me topColor, bottomColor, angle'
        return
    top = sys.argv[1]
    bottom = sys.argv[2]
    angle = float(sys.argv[3])
    rospy.wait_for_service(SERVICE_BEACON_DEBUGGER)
    debugger = rospy.ServiceProxy(SERVICE_BEACON_DEBUGGER, BeaconDebugger)
    status = debugger(top, bottom, angle)
    print 'BeaconDebugger: ', status


if __name__ == '__main__':
    start()