#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      checkForBeacons(cv_image)
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
   pinkBoundaries = [(70, 10, 160), (210, 150, 255)]
   
   im = cv2.imread(image)
   rows, cols, chs = cv2.im.shape
   region = im[0:(rows/2), 0:cols]

   lower = np.array(pinkBoundaries[0], "uint8")
   upper = np.array(pinkBoundaries[1], "uint8")
   mask = cv2.inRange(region, lower, upper)


