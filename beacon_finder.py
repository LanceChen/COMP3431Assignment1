#!/usr/bin/env python

import numpy as np
import operator
import math

from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
from threading import RLock
from sensor_msgs.msg import Image
from assignment1.msg import *
from assignment1.srv import *

from explore import SERVICE_GET_RANGE_FROM_ANGLE
from map_navigator import SERVICE_GENERATE_NAVIGATION_TARGET

SERVICE_BEACON_CHEATER = "beacon_cheater"

TOPIC_BEACONS_IN_CAMERA = 'beacon_list'
SERVICE_STOP_MONITOR_CAMERA = 'stop_monitor_camera'

get_range_from_angle = None
gen_nav_target = None


class BeaconFinder:
    def __init__(self):
        global get_range_from_angle, gen_nav_target
        rospy.init_node('beacon_finder', anonymous=True, log_level=rospy.DEBUG)

        self.stop_monitor = False
        self.found = []
        self.found_list_lock = RLock()
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback)
        self.beacon_pub = rospy.Publisher(TOPIC_BEACONS_IN_CAMERA, BeaconList, queue_size=10)
        rospy.Service(SERVICE_STOP_MONITOR_CAMERA, StopMonitorCamera, self.handle_stop_monitor)
        rospy.Service(SERVICE_BEACON_CHEATER, BeaconCheat, self.handle_beacon_cheat)

        # wait and initialize proxies for all related services
        rospy.wait_for_service(SERVICE_GET_RANGE_FROM_ANGLE)
        rospy.wait_for_service(SERVICE_GENERATE_NAVIGATION_TARGET)
        get_range_from_angle = rospy.ServiceProxy(SERVICE_GET_RANGE_FROM_ANGLE, GetRangeFromAngle)
        gen_nav_target = rospy.ServiceProxy(SERVICE_GENERATE_NAVIGATION_TARGET, GenerateNavigationTarget)

        rospy.loginfo('Image recognition node started')

    def handle_beacon_cheat(self, request):
        """
        @type request: BeaconCheatRequest
        """
        bc = Beacon()
        bc.topColour = request.topColour
        bc.bottomColour = request.bottomColour
        response = BeaconCheatResponse()
        with self.found_list_lock:  # read-write lock for found list
            if not self.is_found(bc):
                alpha = request.angle
                rospy.loginfo("Beacon found at %f" % math.degrees(alpha))
                try:
                    # Get range to beacon
                    # Create navigation target and add to beacon
                    get_range_request = GetRangeFromAngleRequest()
                    get_range_request.angle = alpha
                    beacon_range = get_range_from_angle(get_range_request).range
                    gen_nav_target_request = GenerateNavigationTargetRequest()
                    gen_nav_target_request.range = beacon_range
                    gen_nav_target_request.angle = alpha
                    bc.target = gen_nav_target(gen_nav_target_request).target
                    self.found.append(bc)
                    rospy.loginfo("Beacon located at (%d, %d)" % (bc.target.point.x, bc.target.point.y))
                    response.status = 'Beacon registered'
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
                    response.status = 'Error: %s' % e
            else:
                response.status = 'Beacon already found'
        return response

    def handle_stop_monitor(self, request):
        self.stop_monitor = True
        rospy.loginfo('Image recognition node stopped monitoring camera')
        return StopMonitorCameraResponse()

    def callback(self, data):
        if self.stop_monitor:
            return
        try:
            self.check_for_beacons(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        except CvBridgeError, e:
            rospy.logerr(e)

    def is_found(self, bc):
        for foundBeacons in self.found:
            if bc.topColour == foundBeacons.topColour and bc.bottomColour == foundBeacons.bottomColour:
                return True
        return False

    def check_for_beacons(self, im):
        rospy.loginfo("Processing image")

        # Cut image in half, only process top half
        rows, cols, chs = im.shape
        region = im[0:(rows / 2), 0:cols]

        # BGR lower and upper range for pink
        pink_boundaries = [(120, 60, 210), (210, 150, 255)]

        lower = np.array(pink_boundaries[0], "uint8")
        upper = np.array(pink_boundaries[1], "uint8")

        # binary image, white for in range, black otherwise
        mask = cv2.inRange(region, lower, upper)

        # Finding contours
        # RETR_EXTERNAL for external contours only
        # CHAIN_APPROX_SIMPLE for approximation
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(mask, contours, -1, (128, 255, 0), 3)

        areas = []
        cnt_num = 0

        # List of contour number and areas
        for cnt in contours:
            areas.append((cnt_num, cv2.contourArea(cnt)))
            rospy.loginfo(cv2.contourArea(cnt))
            cnt_num += 1

        # Sort by area
        areas.sort(key=operator.itemgetter(1), reverse=True)

        i = 0
        area_threshold = 50
        beacons_list = []

        # Only look at contours with areas above a certain threshold
        while i < len(areas) and areas[i][1] > area_threshold:
            rospy.loginfo(areas[i][1])
            leftmost = tuple(contours[areas[i][0]][contours[areas[i][0]][:, :, 0].argmin()][0])
            rightmost = tuple(contours[areas[i][0]][contours[areas[i][0]][:, :, 0].argmax()][0])
            topmost = tuple(contours[areas[i][0]][contours[areas[i][0]][:, :, 1].argmin()][0])
            bottommost = tuple(contours[areas[i][0]][contours[areas[i][0]][:, :, 1].argmax()][0])

            left_x = leftmost[0]
            right_x = rightmost[0]
            top_y = topmost[1]
            bottom_y = bottommost[1]

            # Check region above and below pink region
            top_region = region[:top_y, left_x:right_x]
            bottom_region = region[bottom_y:, left_x:right_x]

            bc = self.check_region(top_region, 0)
            if bc.topColour == 'none':
                bc = self.check_region(bottom_region, 1)
            if bc.topColour == 'none':
                rospy.loginfo("False positive for pink")
                i += 1
                continue

            with self.found_list_lock:  # read-write lock for found list
                if not self.is_found(bc):
                    # Calculate angle to centre of beacon
                    d = cols / 2
                    alpha = math.atan((d - (left_x + right_x) / 2) / d * math.tan(math.radians(61.5)))
                    rospy.loginfo("Beacon found at %f" % math.degrees(alpha))
                    try:
                        # Get range to beacon
                        # Create navigation target and add to beacon
                        get_range_request = GetRangeFromAngleRequest()
                        get_range_request.angle = alpha
                        beacon_range = get_range_from_angle(get_range_request).range
                        gen_nav_target_request = GenerateNavigationTargetRequest()
                        gen_nav_target_request.range = beacon_range
                        gen_nav_target_request.angle = alpha
                        bc.target = gen_nav_target(gen_nav_target_request).target
                        beacons_list.append(bc)
                        self.found.append(bc)
                        rospy.loginfo("Beacon located at (%d, %d)" % (bc.target.point.x, bc.target.point.y))
                    except rospy.ServiceException, e:
                        print "Service call failed: %s" % e
                else:
                    pass
            i += 1

        if not rospy.is_shutdown():
            # Publish list of found beacons
            bl = BeaconList()
            bl.foundBeacons = beacons_list
            self.beacon_pub.publish(bl)

    @staticmethod
    # Check region for colour, create beacon object
    def check_region(image, region):
        # BGR lower and upper range for green, blue and yellow
        green_boundaries = [(60, 80, 0), (90, 110, 10)]
        blue_boundaries = [(150, 100, 0), (250, 191, 61)]
        yellow_boundaries = [(0, 140, 160), (70, 240, 255)]

        green_lower = np.array(green_boundaries[0], "uint8")
        green_upper = np.array(green_boundaries[1], "uint8")

        blue_lower = np.array(blue_boundaries[0], "uint8")
        blue_upper = np.array(blue_boundaries[1], "uint8")

        yellow_lower = np.array(yellow_boundaries[0], "uint8")
        yellow_upper = np.array(yellow_boundaries[1], "uint8")

        mask_green = cv2.inRange(image, green_lower, green_upper)
        mask_blue = cv2.inRange(image, blue_lower, blue_upper)
        mask_yellow = cv2.inRange(image, yellow_lower, yellow_upper)

        contours_green, hierarchy_green = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas_green = []

        for cntGreen in contours_green:
            areas_green.append(cv2.contourArea(cntGreen))

        contours_blue, hierarchy_blue = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas_blue = []

        for cntBlue in contours_blue:
            areas_blue.append(cv2.contourArea(cntBlue))

        contours_yellow, hierarchy_yellow = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas_yellow = []

        for cntYellow in contours_yellow:
            areas_yellow.append(cv2.contourArea(cntYellow))

        bc = Beacon()
        # Check maximum area for contours of each colour
        if len(areas_green) != 0 and max(areas_green) > 50:
            if region == 0:
                bc.topColour = 'green'
                bc.bottomColour = 'pink'
            else:
                bc.topColour = 'pink'
                bc.bottomColour = 'green'
        elif len(areas_blue) != 0 and max(areas_blue) > 50:
            if region == 0:
                bc.topColour = 'blue'
                bc.bottomColour = 'pink'
            else:
                bc.topColour = 'pink'
                bc.bottomColour = 'blue'
        elif len(areas_yellow) != 0 and max(areas_yellow) > 50:
            if region == 0:
                bc.topColour = 'yellow'
                bc.bottomColour = 'pink'
            else:
                bc.topColour = 'pink'
                bc.bottomColour = 'yellow'
        else:
            bc.topColour = 'none'
            bc.bottomColour = 'none'

        return bc


def main():
    BeaconFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
