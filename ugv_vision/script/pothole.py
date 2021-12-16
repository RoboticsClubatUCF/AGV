#!/usr/bin/env python3

# Marc Simmonds 
# IGVC 2021  
""" 

    This script is meant to:
    1. Listen for image messages from the depth camera
    2. Find the circles in the image
    3. Calculate the location and area of all circles
    4. Update costmap with location and area of all circles

    Right now, the script gets the image, and displays the edges seen in the image
    using openCV. I need to figure out how to get the location and area of all circles
    in the image, and update the costmap with the location of the obstacles.

"""

import rospy 
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class img_detect:

    def __init__(self):

        # Class members for depth camera image and flag for determining if message is sent
        self.img = None
        self.ready = False

        # Initialize computer vision node
        rospy.init_node("CV_Node", anonymous = False, log_level = rospy.INFO)

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/bowser2/bowser2_dc/image/image", Image, callback=self.img_callback)

        # todo: figure out what message type we're publishing
        # self.location_pub = 
    
    # Simple function for showing current image 
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)


    def img_callback(self, img_msg):
        self.img = img_msg
        self.ready = True

    # Runs the circle detection process
    def run(self):

        # Instantiate Image message to cv2 matrix conversion class
        bridge = CvBridge()
        
        # Runs while the ros is up
        while not rospy.is_shutdown():

            # If the image hasn't been published, we don't do anything
            if not self.ready:
                continue

            # Convert image from ROS image to cv2 image
            cv_img = bridge.imgmsg_to_cv2(self.img)

            # Get a grayscale image 
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

            # Blur image to remove some edges 
            cv_img = cv2.GaussianBlur(cv_img, (3,3), 0)

            # Cut the image in half. 
            cv_img = cv_img[int(len(cv_img)/2) + 20:]

            params = cv2.SimpleBlobDetector_Params()

            ver = (cv2.__version__).split('.')
            if int(ver[0]) < 3 :
                detector = cv2.SimpleBlobDetector(params)
            else : 
                detector = cv2.SimpleBlobDetector_create(params)

            # Run blob detection using cv2 blob detection.
            # detector = cv2.SimpleBlobDetector()
        
            # Get blobs
            poi = detector.detect(cv_img)

            im_with_keypoints = cv2.drawKeypoints(cv_img, poi, np.array([]), (255,0,0), 
                                                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # Show what's going on as a sanity check
            cv2.imshow("fuck marc", im_with_keypoints)
            cv2.waitKey(1)

            self.ready = False

        

def main():

    img_detect_node = img_detect()
    img_detect_node.run()

if __name__=='__main__':

    main()