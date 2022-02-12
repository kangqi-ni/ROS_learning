#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from face_detection_robot.msg import Size

class FaceDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
 
        # Create cv_bridge
        self.bridge = CvBridge()
        self.size = Size()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.size_pub = rospy.Publisher("Size", Size, queue_size=1)
 
        # Get haarcascade files from parameters
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")

        # Initialize haarrcasecade detectors
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)
 
        # Get haar parameters from ros parameters
        self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        self.color = (50, 255, 50)
 
        # Intialize rgb image subscriber
        # The topic name input_rgb_image is remapped to /usb_cam/image_raw in launch file
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
 
    def image_callback(self, data):
        # Use cv_bridge to convert ros image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError(e):
            print(e)
 
        # Create grey image
        grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
 
        # Create historgram to equalize light disturbance
        grey_image = cv2.equalizeHist(grey_image)
 
        # Detect face
        faces_result = self.detect_face(grey_image)
 
        # Create a bbox for the detected face
        if len(faces_result) > 0:
            for face in faces_result:
                x, y, w, h = face
                self.size.width = abs(int(w))
                self.size.height = abs(int(h))
                self.size.x = abs(int(x))
                self.size_pub.publish(self.size)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), self.color, 2)
 
        # Convert opencv image to ros image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
 
    def detect_face(self, input_image):
        # Detect front face
        if self.cascade_1:
            faces = self.cascade_1.detectMultiScale(input_image,
                    self.haar_scaleFactor,
                    self.haar_minNeighbors,
                    cv2.CASCADE_SCALE_IMAGE,
                    (self.haar_minSize, self.haar_maxSize))
 
        # If failed, detect side face
        if len(faces) == 0 and self.cascade_2:
            faces = self.cascade_2.detectMultiScale(input_image,
                    self.haar_scaleFactor,
                    self.haar_minNeighbors,
                    cv2.CASCADE_SCALE_IMAGE,
                    (self.haar_minSize, self.haar_maxSize))
 
        return faces
 
    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()
 
if __name__ == '__main__':
    try:
        # Initialize ros node
        rospy.init_node("face_detector")
        FaceDetector()
        rospy.loginfo("Face detector is started..")
        rospy.loginfo("Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down face detector node.")
        cv2.destroyAllWindows()
