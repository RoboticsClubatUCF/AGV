#!/usr/bin/env python3

# Get data from depth camera
# Find "potholes" 
# Calculate radius of pothole
# Calculate distance from robot
# Publish the location and radius of the pothole 

import rospy
from rospy.core import is_shutdown 
from sensor_msgs.msg import Image
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge

class img_detect:

    def __init__(self):

        self.img = None
        self.ready = False
        
        # Initialize computer vision node
        rospy.init_node("CV_Node", anonymous = False, log_level = rospy.INFO)
        
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/bowser2/bowser2_dc/image/image", Image, callback=self.img_callback)

        # todo: figure out what message typer we're publishing. 
        # self.location_pub = 
    
    def show_image(self, img):
        cv.imshow("Image Window", img)
        cv.waitKey(3)


    def img_callback(self, img_msg):
        self.img = img_msg
        self.ready = True

    def run(self):

        while not rospy.is_shutdown():

            if not self.ready:
                continue

            bridge = CvBridge()
            cv_img = bridge.imgmsg_to_cv2(self.img)
            self.show_image(cv_img)

            self.ready = False

def main():

    img_detect_node = img_detect()
    img_detect_node.run()

if __name__=='__main__':

    main()