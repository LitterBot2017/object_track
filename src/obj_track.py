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
import time


class object_track:

  FORWARD = 1
  SWITCH_CAMERA = 2
  DOWNWARD = 3
  current_state = FORWARD

  bbox = ()
  tracker = cv2.Tracker_create("MIL")
  litter_detected = False
  down_servo = False
  tracker_state = False
  timer = 0

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bbox_pub = rospy.Publisher("bbox",BBox)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("vision/yolo2/image_raw",Image,self.image_callback)
    self.roi_sub = rospy.Subscriber("vision/yolo2/detections",ImageDetections,self.detections_callback)

  def publish_bbox(self,x,y,x_center,y_center,detection,down_servo):
    bbox_msg = BBox()
    bbox_msg.x = x
    bbox_msg.y = y
    bbox_msg.x_center = x_center
    bbox_msg.y_center = y_center
    bbox_msg.detection = detection
    bbox_msg.down_servo = down_servo
    self.bbox_pub.publish(bbox_msg)

  def detections_callback(self,data):
    if self.current_state == self.FORWARD:
      if data.num_detections > 0: 
        if self.tracker_state != True:
          obj = data.detections[0]
          self.litter_detected = True
          self.bbox = (obj.x,obj.y,30,30)
        else:
          self.litter_detected = True
      elif data.num_detections == 0 and self.tracker_state == False:
        self.litter_detected = False
    if self.current_state == self.SWITCH_CAMERA:
      self.litter_detected = False
      return
    if self.current_state == self.DOWNWARD:
      if data.num_detections > 0:
        class_id = -1
        confidence = 0
        for objects in data.detections:
          class_id = objects.class_id
          confidence = objects.confidence
          if class_id < 2 and confidence > 0.3: 
            if self.tracker_state != True:
              self.bbox = (obj.x,obj.y,30,30)
              self.litter_detected = True
              return
            else:
              self.litter_detected = True
              return
        if self.tracker_state != True
          self.litter_detected = False
          return
      else:
        self.litter_detected = False
        return

  def image_callback(self,data):
    print(self.bbox)
    print(self.litter_detected)
    print(self.current_state)
    print(self.tracker_state)
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        print(cv_image.shape)
    except CvBridgeError as e:
      print("hello")
      print(e)

    if self.current_state == self.SWITCH_CAMERA and time.time() - self.timer<3:
      self.publish_bbox(0,0,0,0,False,True)
    elif self.current_state == self.SWITCH_CAMERA and time.time() - self.timer>3:
      self.current_state = self.DOWNWARD

    if self.litter_detected:
      self.track_object(cv_image)
      p1 = (int(self.bbox[0]), int(self.bbox[1]))
      p2 = (int(self.bbox[0] + 30), int(self.bbox[1] + 30))
      cv2.rectangle(cv_image, p1, p2, (0,0,255),4)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def is_centered(self,x,y):
    xdiff = -30000
    ydiff = -20000
    if self.current_state == self.FORWARD:
      xdiff = x-640
      ydiff = y-680      
    elif self.current_state == self.DOWNWARD:
      xdiff = x-640
      ydiff = y-400
    if (abs(xdiff)<10 and abs(ydiff)<10):
      return True
    else:
      return False

  def track_object(self,image_in):
    if self.litter_detected:              
      if self.tracker_state!=True:
        self.tracker = cv2.Tracker_create("MIL")
        ok = self.tracker.init(image_in, self.bbox)
        self.tracker_state = True
        print("Tracker init")

      ok, newbox = self.tracker.update(image_in)
      if ok:
          self.bbox = newbox
          if self.current_state == self.FORWARD:
            if self.is_centered(self.bbox[0],self.bbox[1]):
              self.current_state = self.SWITCH_CAMERA
              self.timer = time.time()
              #Publish switch camera message
            else:
              self.publish_bbox(self.bbox[0],self.bbox[1],640,680,self.litter_detected,False)
          elif self.current_state == self.DOWNWARD:
            if self.is_centered(self.bbox[0],self.bbox[1]):
              self.current_state = self.FORWARD
              #Publish switch camera message          
            else:
              self.publish_bbox(self.bbox[0],self.bbox[1],640,680,self.litter_detected,True)
      else:
        self.tracker_state=False
    else:
      self.tracker_state = False

def main(args):
  ic = object_track()
  rospy.init_node('object_track', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
