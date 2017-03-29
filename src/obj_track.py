#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8
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

  DOWNWARD_IMAGE_HEIGHT = 480
  DOWNWARD_IMAGE_WIDTH = 640
  FORWARD_IMAGE_HEIGHT = 480
  FORWARD_IMAGE_WIDTH = 640

  bbox = ()
  tracker = cv2.Tracker_create("MIL")
  litter_detected = False
  down_servo = False
  tracker_state = False
  timer = time.time()
  bbox2 = () # testing only

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.camera_select_pub = rospy.Publisher("vision/yolo2/camera_select",Int8)
    self.bbox_pub = rospy.Publisher("bbox",BBox)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("vision/yolo2/image_raw",Image,self.image_callback)
    self.roi_sub = rospy.Subscriber("vision/yolo2/detections",ImageDetections,self.detections_callback)

  def select_camera(self,camera):
    msg = Int8();
    msg.data = camera;
    self.camera_select_pub.publish(msg);

  def publish_bbox(self, x, y, x_center, y_center, detection, down_servo):
    bbox_msg = BBox()
    bbox_msg.x = x
    bbox_msg.y = y
    bbox_msg.x_center = x_center
    bbox_msg.y_center = y_center
    bbox_msg.detection = detection
    bbox_msg.down_servo = down_servo
    self.bbox_pub.publish(bbox_msg)

  def handle_forward(self,data):
    #time.time() - self.timer > 1 and
    if  data.num_detections > 0:
      obj = data.detections[0]
      self.bbox2 = (obj.x,obj.y,30,30)
      if self.tracker_state != True:
        obj = data.detections[0]
        self.litter_detected = True
        self.bbox = (obj.x,obj.y,30,30)
      else:
        self.litter_detected = True
    elif data.num_detections == 0 and self.tracker_state == False:
      self.litter_detected = False

  def handle_downward(self,data):
    print("curent state is down")
    if data.num_detections > 0:
      class_id = -1
      confidence = 0
      for objects in data.detections:
        class_id = objects.class_id
        confidence = objects.confidence
        print ("there are unconfirmed detections")
        if class_id < 2 and confidence > 0.3:
          self.bbox2 = (objects.x,objects.y,30,30) 
          if self.tracker_state != True:
            self.bbox = (objects.x,objects.y,30,30)
            print("Reached downward detection lost tracking")
            self.litter_detected = True
            return
          else:
            print("Reached downward detection but not lost tracking")
            self.litter_detected = True
            return
      if self.tracker_state != True:
        self.litter_detected = False
        return
    elif self.tracker_state==False:
      self.litter_detected = False
      return

  def handle_switch(self,data):
    self.litter_detected = False
    self.bbox=()
    self.bbox2=()
    self.tracker_state = False
    if time.time() - self.timer > 1 and data.num_detections>0:
      object_in_switch = data.detections[0]
      if object_in_switch.class_id<2 and object_in_switch.confidence>0.7:
        self.current_state = self.DOWNWARD
        self.bbox=(object_in_switch.x,object_in_switch.y,30,30)
        self.bbox2=(object_in_switch.x,object_in_switch.y,30,30)
        self.litter_detected = True
    return

  def detections_callback(self,data):
    if self.current_state == self.FORWARD:
      self.handle_forward(data)
    elif self.current_state == self.SWITCH_CAMERA:
      self.handle_switch(data)
    elif self.current_state == self.DOWNWARD:
      self.handle_downward(data)
      

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

    if self.current_state == self.SWITCH_CAMERA and time.time() - self.timer<10:
      self.publish_bbox(0,0,0,0,False,True)
    elif self.current_state == self.SWITCH_CAMERA and time.time() - self.timer>10:
      self.current_state = self.FORWARD
      self.bbox = ()
      self.bbox2 = ()

    self.image=cv_image
    if self.litter_detected:
      self.track_object(cv_image)
      p1 = (int(self.bbox[0]), int(self.bbox[1]))
      p2 = (int(self.bbox[0] + 30), int(self.bbox[1] + 30))
      p3 = (int(self.bbox2[0]), int(self.bbox2[1]))
      p4 = (int(self.bbox2[0] + 30), int(self.bbox2[1] + 30))      
      cv2.rectangle(cv_image, p1, p2, (0,0,255),4)
      cv2.rectangle(cv_image, p3, p4, (0,255,0),4)
    elif self.current_state!=self.SWITCH_CAMERA:
      self.tracker_state = False
      self.publish_bbox(0,0,0,0,self.litter_detected,False)
      if self.current_state == self.DOWNWARD:
        self.current_state = self.FORWARD
        self.select_camera(1)
        #self.timer = time.time()
        self.tracker_state=False
        #Publish camera switching message
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def is_centered(self, x, y, width, height):
    if self.current_state==self.FORWARD:
      xdiff = x - (width/2)
      ydiff = y - 400 
    elif self.current_state==self.DOWNWARD:
      xdiff = x - (width/2)
      ydiff = y - (height/2)
    return (abs(xdiff) < 50 and abs(ydiff) < 50)

  def track_object(self,image_in):
    if self.litter_detected:              
      if self.tracker_state!=True:
        self.tracker = cv2.Tracker_create("MIL")
        ok = self.tracker.init(image_in, self.bbox)
        self.tracker_state = True
        print("Tracker init")

      ok, newbox = self.tracker.update(image_in)
      if ok and (abs(self.bbox[0]-newbox[0])<20) and (abs(self.bbox[1]-newbox[1])<20):
          self.bbox = newbox
          if self.current_state == self.FORWARD:
            if self.is_centered(self.bbox[0],self.bbox[1], self.FORWARD_IMAGE_WIDTH, self.FORWARD_IMAGE_HEIGHT):
              self.current_state = self.SWITCH_CAMERA
              self.tracker_state = False
              self.timer = time.time()
              self.select_camera(0)
              #Publish switch camera message
            else:
              self.publish_bbox(self.bbox[0],self.bbox[1],320,400,self.litter_detected,False)
          elif self.current_state == self.DOWNWARD:
            if self.is_centered(self.bbox[0],self.bbox[1], self.DOWNWARD_IMAGE_WIDTH, self.DOWNWARD_IMAGE_HEIGHT):
              self.current_state = self.FORWARD
              self.bbox = ()
              self.bbox2 =()
              self.select_camera(1) 
              #self.timer = time.time()
              self.tracker_state=False
              #Publish switch camera message          
            else:
              self.publish_bbox(self.bbox[0],self.bbox[1],320,135,self.litter_detected,True)
      else:
        self.tracker_state=False

def main(args):
  ic = object_track()
  rospy.init_node('object_track', anonymous=True)
  ic.select_camera(1)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
