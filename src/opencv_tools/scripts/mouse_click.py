#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('opencv_tools')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt16MultiArray
from reetiros.msg import reetiNeckPose

class image_converter:

  x = 50
  y = 50

  refPt = []
  cropping = False

  
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image, self.callback)
    self.point_pub = rospy.Publisher("clicked_point", UInt16MultiArray, queue_size = 1)
    self.neck_clicked_pub = rospy.Publisher("/reeti/neck", reetiNeckPose, queue_size = 1)

  def click_callback(self, event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        msg = UInt16MultiArray()
        msg.data = [x, y]
        self.point_pub.publish(msg)
        rospy.loginfo("(%d, %d) is clicked ", x, y)
        neck_msg = reetiNeckPose()
        neck_msg.neckYaw = (x * -0.1) + 85
        neck_msg.neckPitch = 50
        neck_msg.neckRoll = (y * -0.3) + 133
        self.neck_clicked_pub.publish(neck_msg)
        
    
  def callback(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    clone = cv_image.copy()
        
    cv2.namedWindow("Image window for clicking")
    cv2.imshow("Image window for clicking", cv_image)
    cv2.waitKey(3)

    cv2.setMouseCallback("Image window for clicking", self.click_callback)
    # if there are two reference points, then crop the region of interest
    # from teh image and display it
    # if len(self.refPt) == 2:
    #     # draw a rectangle around the region of interest

    #     roi = clone[self.refPt[0][1]:self.refPt[1][1], self.refPt[0][0]:self.refPt[1][0]]
    #     cv2.imshow("ROI", roi)
    #     #cv2.waitKey(3)

    #cv2.rectangle(clone, self.refPt[0], self.refPt[1], (0, 255, 0), 2)
    #cv2.imshow("Image window for clicking", clone)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
