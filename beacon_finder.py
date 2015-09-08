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

TOPIC_BEACONS_IN_CAMERA = 'beacon_list'
SERVICE_STOP_MONITOR_CAMERA = 'stop_monitor_camera'


class BeaconFinder:
    def __init__(self):
        rospy.init_node('beacon_finder', anonymous=True, log_level=rospy.DEBUG)
        cv2.namedWindow("Image window", 1)
        self.stop_monitor = False
        self.found = []
        self.bridge = CvBridge()
        # noinspection PyTypeChecker
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback)
        self.beacon_pub = rospy.Publisher(TOPIC_BEACONS_IN_CAMERA, BeaconList, queue_size=10)
        rospy.Service(SERVICE_STOP_MONITOR_CAMERA, StopMonitorCamera, self.handle_stop_monitor)
        # wait and initialize proxies for all related services
        rospy.wait_for_service(SERVICE_GET_RANGE_FROM_ANGLE)
        rospy.wait_for_service(SERVICE_GENERATE_NAVIGATION_TARGET)
        self.get_range_from_angle = rospy.ServiceProxy(SERVICE_GET_RANGE_FROM_ANGLE, GetRangeFromAngle)
        self.gen_nav_target = rospy.ServiceProxy(SERVICE_GENERATE_NAVIGATION_TARGET, GenerateNavigationTarget)
        rospy.loginfo('Image recognition node started')
        self.processing_lock = RLock()
        self.processing = False
        self.latestImage = None

    # noinspection PyUnusedLocal
    def handle_stop_monitor(self, request):
        self.stop_monitor = True
        rospy.loginfo('Image recognition node stopped monitoring camera')
        return StopMonitorCameraResponse()

    def callback(self, data):
        if self.stop_monitor:
            return
        try:
            self.latestImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if not self.processing:
                self.processing = True
                try:
                    self.check_for_beacons(self.latestImage)
                except CvBridgeError, e:
                    pass
                self.processing = False
        except CvBridgeError, e:
            print e

    def is_found(self, bc):
        for foundBeacons in self.found:
            if bc.topColour == foundBeacons.topColor and bc.bottomColour == foundBeacons.bottomColor:
                return True
        return False

    def check_for_beacons(self, im):
        with self.processing_lock:
            rospy.loginfo("Processing image")
            # images = ['test0.jpg', 'test1.jpg', 'test2.jpg', 'test3.jpg', 'test4.jpg']
            # for i in images:
            #im = cv2.imread(image)
            rows, cols, chs = im.shape
            region = im[0:(rows / 2), 0:cols]
            pink_boundaries = [(120, 60, 210), (210, 150, 255)]

            lower = np.array(pink_boundaries[0], "uint8")
            upper = np.array(pink_boundaries[1], "uint8")

            mask = cv2.inRange(region, lower, upper)
            cv2.namedWindow("image", cv2.cv.CV_WINDOW_NORMAL)
            cv2.imshow("image", np.hstack([mask]))
            #cv2.waitKey(0)

            # output = cv2.bitwise_and(im, im, mask = mask)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(mask, contours, -1, (128, 255, 0), 3)
            cv2.namedWindow("image", cv2.cv.CV_WINDOW_NORMAL)
            cv2.imshow("image", np.hstack([mask]))
            #cv2.waitKey(0)

            # print (len(contours))

            areas = []
            cnt_num = 0

            for cnt in contours:
                # print (cv2.contourArea(cnt))
                areas.append((cnt_num, cv2.contourArea(cnt)))
                cnt_num += 1

            areas.sort(key=operator.itemgetter(1), reverse=True)
            # print(areas)

            i = 0
            area_threshold = 1000
            beacons_list = []
            rospy.loginfo("Area index:")
            while i < len(areas) and areas[i][1] > area_threshold:
                leftmost = tuple(contours[areas[i][0]][contours[areas[i][0]][:, :, 0].argmin()][0])
                rightmost = tuple(contours[areas[i][0]][contours[areas[i][0]][:, :, 0].argmax()][0])
                topmost = tuple(contours[areas[i][0]][contours[areas[i][0]][:, :, 1].argmin()][0])
                bottommost = tuple(contours[areas[i][0]][contours[areas[i][0]][:, :, 1].argmax()][0])
                print(areas[i][1])

                print leftmost
                print rightmost
                print topmost
                print bottommost

                left_x = leftmost[0]
                right_x = rightmost[0]
                top_y = topmost[1]
                bottom_y = bottommost[1]

                top_region = im[top_y - 300:top_y, left_x:right_x]
                bottom_region = im[bottom_y:bottom_y + 300, left_x:right_x]

                bc = self.check_region(top_region, 0)
                if bc.topColour == 'none':
                    bc = self.check_region(bottom_region, 1)

                if not self.is_found(bc):
                    # calculate x value
                    d = cols / 2
                    alpha = math.atan((d - cols) / d * math.tan(math.radians(61.5)))
                    try:
                        get_range_request = GetRangeFromAngleRequest()
                        get_range_request.angle = alpha
                        beacon_range = self.get_range_from_angle(get_range_request).range
                        gen_nav_target_request = GenerateNavigationTargetRequest()
                        gen_nav_target_request.range = beacon_range
                        gen_nav_target_request.angle = alpha
                        bc.target = self.gen_nav_target(gen_nav_target_request).target
                        beacons_list.append(bc)
                        self.found.append(bc)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s" % e
                else:
                    pass
                i += 1
                rospy.loginfo("Increment: %d" % i)

            if not rospy.is_shutdown():
                bl = BeaconList()
                bl.foundBeacons = beacons_list
                # rospy.loginfo(bl)
                self.beacon_pub.publish(bl)
                # rate.sleep()

    @staticmethod
    def check_region(image, region):
        green_boundaries = [(60, 80, 0), (90, 110, 10)]
        blue_boundaries = [(150, 100, 0), (250, 191, 61)]
        yellow_boundaries = [(0, 140, 160), (70, 240, 255)]

        green_lower = np.array(green_boundaries[0], "uint8")
        green_upper = np.array(green_boundaries[1], "uint8")

        # cv2.namedWindow("green mask", cv2.cv.CV_WINDOW_NORMAL)
        # cv2.imshow("green mask", np.hstack([mask2]))
        # cv2.waitKey(0)

        blue_lower = np.array(blue_boundaries[0], "uint8")
        blue_upper = np.array(blue_boundaries[1], "uint8")

        # cv2.namedWindow("blue mask", cv2.cv.CV_WINDOW_NORMAL)
        # cv2.imshow("blue mask", np.hstack([mask3]))
        # cv2.waitKey(0)

        yellow_lower = np.array(yellow_boundaries[0], "uint8")
        yellow_upper = np.array(yellow_boundaries[1], "uint8")

        # cv2.namedWindow("yellow mask", cv2.cv.CV_WINDOW_NORMAL)
        # cv2.imshow("yellow mask", np.hstack([mask4]))
        # cv2.waitKey(0)

        mask_green = cv2.inRange(image, green_lower, green_upper)
        mask_blue = cv2.inRange(image, blue_lower, blue_upper)
        mask_yellow = cv2.inRange(image, yellow_lower, yellow_upper)

        contours_green, hierarchy_green = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas_green = []
        cnt_num_green = 0

        for cntGreen in contours_green:
            # print (cv2.contourArea(cnt))
            areas_green.append((cnt_num_green, cv2.contourArea(cntGreen)))
            cnt_num_green += 1

        contours_blue, hierarchy_blue = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas_blue = []
        cnt_num_blue = 0

        for cntBlue in contours_blue:
            # print (cv2.contourArea(cnt))
            areas_blue.append((cnt_num_blue, cv2.contourArea(cntBlue)))
            cnt_num_blue += 1

        contours_yellow, hierarchy_yellow = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas_yellow = []
        cnt_num_yellow = 0

        for cntYellow in contours_yellow:
            # print (cv2.contourArea(cnt))
            areas_yellow.append((cnt_num_yellow, cv2.contourArea(cntYellow)))
            cnt_num_yellow += 1

        bc = Beacon()
        if max(areas_green, key=operator.itemgetter(1)) > 1000:
            if region == 0:
                bc.topColour = 'green'
                bc.bottomColour = 'pink'
            else:
                bc.topColour = 'pink'
                bc.bottomColour = 'green'
        elif max(areas_blue, key=operator.itemgetter(1)) > 1000:
            if region == 0:
                bc.topColour = 'blue'
                bc.bottomColour = 'pink'
            else:
                bc.topColour = 'pink'
                bc.bottomColour = 'blue'
        elif max(areas_yellow, key=operator.itemgetter(1)) > 1000:
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
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)


def main():
    BeaconFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
