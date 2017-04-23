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

  DOWNWARD_IMAGE_HEIGHT = 360
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
    self.arm_sub = rospy.Subscriber("arm_state", String, self.arm_state_callback)
    self.image_sub = rospy.Subscriber("vision/yolo2/image_raw",Image,self.image_callback)
    self.roi_sub = rospy.Subscriber("vision/yolo2/detections",ImageDetections,self.detections_callback)

  def select_camera(self,camera):
    msg = Int8();
    msg.data = camera;
    self.camera_select_pub.publish(msg);

  def publish_bbox(self, x, y, x_center, y_center, detection, down_servo, ready_for_pickup):
    bbox_msg = BBox()
    bbox_msg.x = x
    bbox_msg.y = y
    bbox_msg.x_center = x_center
    bbox_msg.y_center = y_center
    bbox_msg.detection = detection
    bbox_msg.down_servo = down_servo
    bbox_msg.ready_for_pickup = ready_for_pickup
    self.bbox_pub.publish(bbox_msg)

  def arm_state_callback(self,msg):
    if msg.data == "in_progress":
      self.publish_bbox(self.bbox[0],self.bbox[1],320,240,self.litter_detected,True,True)
      return
    elif msg.data == "pickup_done":
      self.current_state = self.FORWARD
      self.bbox = ()
      self.bbox2 =()
      self.litter_detected = False
      self.select_camera(1) 
      self.tracker_state=False

  def handle_forward(self,data):
    #time.time() - self.timer > 1 and
    if  data.num_detections > 0:
      obj = data.detections[0]
      self.bbox2 = ((obj.x - obj.width/2), (obj.y - obj.height/2), obj.width, obj.height)
      self.litter_detected = True
      if self.tracker_state == False:
        self.bbox = ((obj.x - obj.width/2), (obj.y - obj.height/2), obj.width, obj.height)
    elif data.num_detections == 0 and self.tracker_state == False:
      self.litter_detected = False

  def is_can_bottle(self, data):
    if data.num_detections > 0:
      class_id = -1
      confidence = 0
      for objects in data.detections:
        class_id = objects.class_id
        confidence = objects.confidence
        print ("there are unconfirmed detections")
        if class_id < 2 and confidence > 0.5:
          x = objects.x - objects.width/2 
          y = objects.y - objects.height/2 
          width = objects.width
          height = objects.height
          is_litter = True
          return (x,y,width,height,is_litter)
    return (0,0,0,0,False)

  def handle_downward(self,data):
    print("curent state is down")
    x,y,width,height,is_litter = self.is_can_bottle(data)    
    if is_litter:
      self.bbox2 = (x,y,width,height)
      self.litter_detected = True 
      if self.tracker_state == False:
        self.bbox = (x,y,width,height)
        print("Reached downward detection lost tracking")
        return
    elif self.tracker_state==False:
      self.litter_detected = False
      return

  def handle_switch(self,data):
    self.litter_detected = False
    self.bbox=()
    self.bbox2=()
    self.tracker_state = False
    x,y,width,height,is_litter = self.is_can_bottle(data)
    if time.time() - self.timer > 1 and is_litter:
      self.current_state = self.DOWNWARD
      self.bbox=(x,y,width,height)
      self.bbox2=(x,y,width,height)
      self.litter_detected = True

  def detections_callback(self,data):
    if self.current_state == self.FORWARD:
      self.handle_forward(data)
    elif self.current_state == self.SWITCH_CAMERA:
      self.handle_switch(data)
    elif self.current_state == self.DOWNWARD:
      self.handle_downward(data)
  
  def draw_rectangle (self, cv_image):
    if self.bbox:
      p1 = (int(self.bbox[0]), int(self.bbox[1]))
      p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
      p3 = (int(self.bbox2[0]), int(self.bbox2[1]))
      p4 = (int(self.bbox2[0] + self.bbox2[2]), int(self.bbox2[1]  + self.bbox2[3]))
      if self.bboxInBounds(self.bbox) ==False or self.bboxInBounds(self.bbox2) == False:
        return cv_image
      cv2.rectangle(cv_image, p1, p2, (0,0,255),4)
      cv2.rectangle(cv_image, p3, p4, (0,255,0),4)
    return cv_image    

  def image_callback(self,data):
    print("bbox"+str(self.bbox))
    print("Litter detect"+str(self.litter_detected))
    print("Curr_state"+str(self.current_state))
    print("tracker_state"+str(self.tracker_state))
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        print(cv_image.shape)
    except CvBridgeError as e:
      print("hello")
      print(e)

    if self.current_state == self.SWITCH_CAMERA and time.time() - self.timer<10:
      self.publish_bbox(0,0,0,0,False,True,False)
    elif self.current_state == self.SWITCH_CAMERA and time.time() - self.timer>10:
      self.current_state = self.FORWARD
      self.select_camera(1)
      self.bbox = ()
      self.bbox2 = ()
      self.tracker_state = False
      self.litter_detected = False
    if self.litter_detected:
      self.track_object(cv_image)
      cv_image=self.draw_rectangle(cv_image)
    elif self.current_state!=self.SWITCH_CAMERA:
      self.tracker_state = False
      self.publish_bbox(0,0,0,0,self.litter_detected,False,False)
      if self.current_state == self.DOWNWARD:
        self.current_state = self.FORWARD
        self.select_camera(1)
        self.tracker_state=False
        #Publish camera switching message
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

  def is_centered(self, x, y, width, height):
    if self.current_state==self.FORWARD:
      xdiff = x - (width/2)
      ydiff = y - (height - 80) 
    elif self.current_state==self.DOWNWARD:
      xdiff = x - (width/2)
      ydiff = y - (3*height/4)
    return (abs(xdiff) < 50 and abs(ydiff) < 50)

  def bboxInBounds(self,bbox):
    p1 = (int(bbox[0]), int(bbox[1]))
    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
    if p1[0] >= 640 or p1[1] >= 480 or p1[0] <= 0 or p1[1] <= 0:
      return False
    if p2[0] >= 640 or p2[1] >= 480 or p2[0] <= 0 or p2[1] <= 0:
      return False
    return True

  yoloCounter = 0

  def isBBoxStable(self,newBBox, prevBBox, yoloBBox):
    stability = False
    if prevBBox==():
      return False
    if (abs(prevBBox[0]-newBBox[0])<20) and (abs(prevBBox[1]-newBBox[1])<20):
      stability = True
    if (abs(yoloBBox[0]-newBBox[0])<20) and (abs(yoloBBox[1]-newBBox[1])<20):
      stability = True
      self.yoloCounter = 0
    elif self.yoloCounter<4:
      self.yoloCounter+=1
    else:
      stability = False
      self.yoloCounter = 0
    return stability

  def getBBoxCenter(self, bbox):
    x = bbox[0]+(bbox[2]/2)
    y = bbox[1]+(bbox[3]/2)
    return x,y

  def track_object(self,image_in):
    if self.bbox!=():
      if self.tracker_state!=True and self.bboxInBounds(self.bbox) == True:
        self.tracker = cv2.Tracker_create("MIL")
        ok = self.tracker.init(image_in, self.bbox)
        self.tracker_state = True
      ok, newbox = self.tracker.update(image_in)
      # bboxX = self.bbox[0]
      # bboxY = self.bbox[1]
      # newBboxX = newbox[0]
      # newBboxY = newbox[1]
      if ok and self.isBBoxStable(newbox, self.bbox, self.bbox2):
          self.bbox = newbox
          x,y=self.getBBoxCenter(self.bbox)
          if self.current_state == self.FORWARD:
            if self.is_centered(x,y, self.FORWARD_IMAGE_WIDTH, self.FORWARD_IMAGE_HEIGHT):
              self.current_state = self.SWITCH_CAMERA
              self.timer = time.time()
              self.select_camera(0)
              #Publish switch camera message
            else:
              self.publish_bbox(x,y,(self.FORWARD_IMAGE_WIDTH/2),(self.FORWARD_IMAGE_HEIGHT - 80),self.litter_detected,False,False)
          elif self.current_state == self.DOWNWARD: 
            if self.is_centered(x,y, self.DOWNWARD_IMAGE_WIDTH, self.DOWNWARD_IMAGE_HEIGHT):
              self.publish_bbox(x,y, (self.DOWNWARD_IMAGE_WIDTH/2),(3*self.DOWNWARD_IMAGE_HEIGHT/4),self.litter_detected,True,True)
            else:
              self.publish_bbox(x,y,(self.DOWNWARD_IMAGE_WIDTH/2),(3*self.DOWNWARD_IMAGE_HEIGHT/4),self.litter_detected,True,False)
              #Publish switch camera message
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
