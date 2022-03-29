#!/usr/bin/env python3
# Marc Simmonds
# IGVC 2022
""" 

    This script is meant to:
    1. Listen for image messages from the depth camera
    2. Find the potholes in the image
    3. Get the locations of the potholes using the depth image
    4. Publish these locations to the costmap... somehow...
    
    I think this works, but I need to test it

"""

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
import cv2
from cv_bridge import CvBridge
import math
import numpy as np

class pothole_detect:

    ERR = -100

    def __init__(self):

        # Class members for depth camera image and flag for determining if message is sent
        self.img = None
        self.ready_img = False
        self.ready_depth = False
        self.depth_img = None
        self.bridge = CvBridge()
        self.frame = "bowser2/camera_link"

        rospy.init_node("Detected Obstacles", anonymous = False, log_level = rospy.INFO)

        self.image_sub = rospy.Subscriber("/bowser2/bowser2_dc/image/image", Image, callback=self.img_callback)
 
        self.depth_sub = rospy.Subscriber("/bowser2/bowser2_dc/depth/image", Image, callback = self.depth_callback)

        self.pub = rospy.Publisher("Potholes", Polygon, queue_size = 3)
    
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(1)

    def img_callback(self, img_msg):
        self.img = self.bridge.imgmsg_to_cv2(img_msg)
        self.ready_img = True

    def depth_callback(self, depth_msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(depth_msg)
        self.ready_depth = True

    def isPothole(self, perimeter, area):
        
        if perimeter == 0 or area == 0:
            return False

        circularity = (4*math.pi*area)/(math.pow(perimeter,2))

        if circularity > 0.5 and (75 < area < 20000):
            return True

    # The angular distance is correct and the trig checks out, but this still doesn't work correctly for some reason. 
    def getLocation(self, coordinate, depth):
        
        if depth > 19:
            return self.ERR
        
        # The horizontal angular distance of the point is equal to the angular distance per pixel
        # multiplied by the horizontal distance of the pixel from the center of the frame.
        theta = ((45/640) * abs(coordinate[0] - 320))

        # Point is in center of frame
        if theta == 0:
            return 0

        # Since the depth is the Z distance outward from the camera as a straight line
        # the horizontal distance of the point is the arctangent of the angular distance
        # multiplied by the depth: 
        # See: tan(theta) = horizontal_dist/depth

        c = depth/math.cos(math.radians(theta))
        
        xdist = math.sqrt((c*c) - (depth*depth))

        if (coordinate[0] - 320) < 0:
            xdist*=-1

        return xdist

    # Runs the circle detection process
    def getPotholes(self):
        
        while not rospy.is_shutdown():

            # If the image hasn't been published, we don't do anything
            if (not self.ready_img) and (not self.ready_depth):
                continue

            cv_img = self.img

            ret, cv_img = cv2.threshold(cv_img, 125, 255, 0)

            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            
            # Find the contours in the image
            contours, hierarchy = cv2.findContours(cv_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Filter for contours with the desired area and circularity
            
            validContours = []
            for i in contours:
                
                area = cv2.contourArea(i)
                perimeter = cv2.arcLength(i, True)
                
                if self.isPothole(perimeter, area):
                    validContours.append(i)
             
            if not len(validContours) <= 0:
                # Polygon
                for contour in validContours:
                    p = PolygonStamped()
                    p.header = self.frame
                    # Point
                    for point in contour:
                        point = np.squeeze(point)
                        try:
                            # Try to get the point in the contour in the depth image
                            # Right now, this just prints the location. Sometimes it prints the max range of the sensor, 
                            # which means it isn't detecting anything. Other times, weird shit happens.
                            xPoint = self.getLocation(point, self.depth_img[point[1]][point[0]])
                            zPoint = self.depth_img[point[1]][point[0]]
                            p.polygon.points = Point32(x=xPoint, y=0.000, z = zPoint)
                            # Prints (z, x). y is assumed to be the ground plane.
                            #print(self.depth_img[point[1]][point[0]], self.getLocation(point, self.depth_img[point[1]][point[0]]))
                        
                        except IndexError:
                            continue
                        
                        except TypeError:
                            continue
                    self.pub.publish(p)


            # Show what's going on as a sanity check

            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
            
            cv2.drawContours(cv_img, validContours, -1, (0,250,0), 3)

            self.show_image(cv_img)
            self.ready_img = False
            self.ready_depth = False

def main():

    img_detect_node = pothole_detect()
    img_detect_node.getPotholes()

if __name__=='__main__':

    main()