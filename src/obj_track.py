#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from object_tracker.msg import BBox
from yolo2.msg import ImageDetections
from yolo2.msg import Detection
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

  init_once = False
  bbox = ()
  tracker = cv2.Tracker_create("MIL")
  count = 0
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bbox_pub = rospy.Publisher("bbox",BBox)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("vision/image_raw",Image,self.image_callback)
    self.roi_sub = rospy.Subscriber("vision/yolo2/detections",ImageDetections,self.detections_callback)

  def detections_callback(self,data):
    if not self.init_once:
      if len(data.detections)>0:
        detected=data.detections[0]
        self.bbox=(detected.x,detected.y,detected.width/2,detected.height/2)
      else:
        self.bbox=()

  def image_callback(self,data):

    self.count=self.count+1
    if(self.count%30==0):
      self.init_once=False
      self.count=0
      return
    if len(self.bbox)>0:              
      try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        print(cv_image.shape)
      except CvBridgeError as e:
        print("hello")
        print(e)

      if not self.init_once:
        self.tracker=cv2.Tracker_create("MIL")
        ok = self.tracker.init(cv_image, self.bbox)
        self.init_once = True

      ok, newbox = self.tracker.update(cv_image)

      if ok:
          print(self.bbox)
          print(newbox)
          p1 = (int(newbox[0]), int(newbox[1]))
          p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
          msg_to_send = BBox()
          msg_to_send.x=int(newbox[0]+(newbox[2]/2))
          msg_to_send.y=int(newbox[1]+(newbox[3]/2))
          self.bbox_pub.publish(msg_to_send)
          cv2.rectangle(cv_image, p1, p2, (0,0,255),6)
      else:
        self.init_once=False

      (rows,cols,channels) = cv_image.shape

      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)

      try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      except CvBridgeError as e:
        print(e)

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
