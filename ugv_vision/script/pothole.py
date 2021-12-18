#!/usr/bin/env python3

# Marc Simmonds 
# IGVC 2021  
""" 

    This script is meant to:
    1. Listen for image messages from the depth camera
    2. Find the circles in the image
    3. Get the locations of the circles
    4. Update costmap with location and area of all circles

    Right now, the script finds and highlights the circles. We'll need to use the depth camera to 
    get the actual locations.

"""

import cv_bridge
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import cv2
from cv_bridge import CvBridge
import math
import numpy as np

class img_detect:

    def __init__(self):

        # Class members for depth camera image and flag for determining if message is sent
        self.img = None
        self.ready_img = False
        self.ready_depth = False
        self.depth_img = None 
        self.bridge = CvBridge()

        # Initialize computer vision node
        rospy.init_node("CV_Node", anonymous = False, log_level = rospy.INFO)

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/bowser2/bowser2_dc/image/image", Image, callback=self.img_callback)

        # Subscribe to depth image topic 
        self.depth_sub = rospy.Subscriber("/bowser2/bowser2_dc/depth/image", Image, callback = self.depth_callback)

        # todo: figure out what message type we're publishing
        # self.location_pub = 
    
    # Simple function for showing current image 
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)


    def img_callback(self, img_msg):
        self.img = img_msg
        self.ready_img = True

    def depth_callback(self, depth_msg):
        # self.depth_img = depth_msg
        self.depth_img = self.bridge.imgmsg_to_cv2(depth_msg)
        # print(self.depth_img)
        self.ready_depth = True


    # Helper method to calculate circularity     
    def get_circularity(self, perimeter, area):
        
        if perimeter == 0:
            return 0

        if area == 0:
            return 0

        return (4*math.pi*area)/(math.pow(perimeter,2))

    # Runs the circle detection process
    def run(self):

        # Instantiate Image message to cv2 matrix conversion class
        bridge = CvBridge()
        
        # Runs while the ros is up
        while not rospy.is_shutdown():

            # If the image hasn't been published, we don't do anything
            if (not self.ready_img) and (not self.ready_depth):
                continue

            # Convert image from ROS image to cv2 image
            cv_img = bridge.imgmsg_to_cv2(self.img)

            # Get a grayscale image 
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

            # Blur image to remove some edges 
            cv_img = cv2.GaussianBlur(cv_img, (3,3), 0)

            # Cut the image in half. 
            #cv_img = cv_img[int(len(cv_img)/2) + 20:]

            # Apply simple thresholding to the image, to remove everything that isn't close to white. 
            ret, cv_img = cv2.threshold(cv_img, 125, 255, 0) 
            
            # Find the contours in the image
            contours, hierarchy = cv2.findContours(cv_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Filter for contours with the desired area and circularity
            validContours = []

            for i in contours:
                
                # Get the area and perimeter, then use it to calculate circularity.
                area = cv2.contourArea(i)
                perimeter = cv2.arcLength(i, True)
                circularity = self.get_circularity(perimeter, area)
                
                #  Add to contour array if the contour is circular, and the contour is the size that we want.
                if 100 < area < 20000 and circularity > 0.4:
                    validContours.append(i)
            
            if not len(validContours) <= 0:
                for contour in validContours:
                    print("------------------------------")
                    for point in contour:
                        point = np.squeeze(point)
                        try:
                            print(self.depth_img[point[1]][point[0]])
                        except IndexError:
                            print("Index Err: {} {}".format(point[1], point[0]))
                        except TypeError:
                            print("fuck you")

            # Convert the image to color, so we can draw on it properly
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
            
            # Draw the contours 
            cv2.drawContours(cv_img, validContours, -1, (0,250,0), 3)

            # Find the location in the depth image
            
            # Show what's going on as a sanity check
            self.show_image(cv_img)
            self.ready_img = False
            self.ready_depth = False

        

def main():

    img_detect_node = img_detect()
    img_detect_node.run()

if __name__=='__main__':

    main()
