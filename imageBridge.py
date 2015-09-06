#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import operator
import math
import beacons

class image_converter:

  def __init__(self):

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      beaconsList = checkForBeacons(cv_image)
    except CvBridgeError, e:
      print e

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)




def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

def checkForBeacons(image):
   #images = ['test0.jpg', 'test1.jpg', 'test2.jpg', 'test3.jpg', 'test4.jpg']
   #for i in images:
   im = cv2.imread(image)
   rows, cols, chs = cv2.im.shape
   region = im[0:(rows/2), 0:cols]
   pinkBoundaries = [(120, 60, 210), (210, 150, 255)]

   lower = np.array(pinkBoundaries[0], "uint8")
   upper = np.array(pinkBoundaries[1], "uint8")

   mask = cv2.inRange(region, lower, upper)
   cv2.namedWindow("image", cv2.cv.CV_WINDOW_NORMAL)
   cv2.imshow("image", np.hstack([mask]))
   cv2.waitKey(0)


   #output = cv2.bitwise_and(im, im, mask = mask)
   contours, heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   cv2.drawContours(mask, contours, -1, (128, 255, 0), 3)
   cv2.namedWindow("image", cv2.cv.CV_WINDOW_NORMAL)
   cv2.imshow("image", np.hstack([mask]))
   cv2.waitKey(0)

   #print (len(contours))

   areas = []
   cntNum = 0

   for cnt in contours:
      #print (cv2.contourArea(cnt))
      areas.append((cntNum, cv2.contourArea(cnt)))
      cntNum = cntNum + 1

   areas.sort(key=operator.itemgetter(1), reverse=True)
   #print(areas)

   i = 0
   area_threshold = 100
   beaconsList = []

   while areas[i][1] > area_threshold:
      leftmost = tuple(contours[i][contours[i][:,:,0].argmin()][0])
      rightmost = tuple(contours[i][contours[i][:,:,0].argmax()][0])
      topmost = tuple(contours[i][contours[i][:,:,1].argmin()][0])
      bottommost = tuple(contours[i][contours[i][:,:,1].argmax()][0])
      print(areas[i][1])
      
      print leftmost
      print rightmost
      print topmost
      print bottommost

      leftx = leftmost[0]
      rightx = rightmost[0]
      topy = topmost[1]
      bottomy = bottommost[1]

      topRegion = im[topy-300:topy, leftx:rightx]
      bottomRegion = im[bottomy:bottomy+300, leftx:rightx]

      bc = checkRegion(topRegion, 0)
      if bc.top == 'none':
         bc = checkRegion(bottomRegion, 1)
      
      if beaconFound(bc) == False:
         #calculate x value
         d = cols/2
         alpha = math.atan((cols-d)/d*math.tan(math.radians(61.5)))
         x,y = locateBeacon(alpha)#call explore service to find beacon location
         bc.x = x
         bc.y = y
         beaconsList.append(bc)
      else:
         pass

      i = i + 1

   return beaconsList


def checkRegion(image, region):
   greenBoundaries = [(60, 80, 0), (90, 110, 10)]
   blueBoundaries = [(150, 100, 0), (250, 191, 61)]
   yellowBoundaries = [(0, 140, 160), (70, 240, 255)]

   greenLower = np.array(greenBoundaries[0], "uint8")
   greenUpper = np.array(greenBoundaries[1], "uint8")
   
   #cv2.namedWindow("green mask", cv2.cv.CV_WINDOW_NORMAL)
   #cv2.imshow("green mask", np.hstack([mask2]))
   #cv2.waitKey(0)

   blueLower = np.array(blueBoundaries[0], "uint8")
   blueUpper = np.array(blueBoundaries[1], "uint8")
   
   #cv2.namedWindow("blue mask", cv2.cv.CV_WINDOW_NORMAL)
   #cv2.imshow("blue mask", np.hstack([mask3]))
   #cv2.waitKey(0)

   yellowLower = np.array(yellowBoundaries[0], "uint8")
   yellowUpper = np.array(yellowBoundaries[1], "uint8")
   
   #cv2.namedWindow("yellow mask", cv2.cv.CV_WINDOW_NORMAL)
   #cv2.imshow("yellow mask", np.hstack([mask4]))
   #cv2.waitKey(0)

   maskGreen = cv2.inRange(image, greenLower, greenUpper)
   maskBlue = cv2.inRange(image, blueLower, blueUpper)
   maskYellow = cv2.inRange(image, yellowLower, yellowUpper)

   contoursGreen, heirarchyGreen = cv2.findContours(maskGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   areasGreen = []
   cntNumGreen = 0

   for cntGreen in contoursGreen:
      #print (cv2.contourArea(cnt))
      areasGreen.append((cntNumGreen, cv2.contourArea(cntGreen)))
      cntNumGreen = cntNumGreen + 1
   
   contoursBlue, heirarchyBlue = cv2.findContours(maskBlue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   areasBlue = []
   cntNumBlue = 0

   for cntBlue in contoursBlue:
      #print (cv2.contourArea(cnt))
      areasBlue.append((cntNumBlue, cv2.contourArea(cntBlue)))
      cntNumBlue = cntNumBlue + 1

   contoursYellow, heirarchYellow = cv2.findContours(maskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   areasYellow = []
   cntNumYellow = 0

   for cntYellow in contoursYellow:
      #print (cv2.contourArea(cnt))
      areasYellow.append((cntNumYellow, cv2.contourArea(cntYellow)))
      cntNumYellow = cntNumYellow + 1
   
   bc = Beacons()
   if max(areasGreen, key=operator.itemgetter(1)) > 1000:
      if region == 0:      
         bc.top = 'green'
         bc.bottom = 'pink'
      else:
         bc.top = 'pink'
         bc.bottom = 'green'
   elif max(areasBlue, key=operator.itemgetter(1)) > 1000:
      if region == 0:
         bc.top = 'blue'
         bc.bottom = 'pink'
      else:
         bc.top = 'pink'
         bc.bottom = 'blue'
   elif max(areasYellow, key=operator.itemgetter(1)) > 1000:
      if region == 0:      
         bc.top = 'yellow'
         bc.bottom = 'pink'
      else:
         bc.top = 'pink'
         bc.bottom = 'yellow'
   else:
      bc.top = 'none'
      bc.bottom = 'none'

   return bc
   


