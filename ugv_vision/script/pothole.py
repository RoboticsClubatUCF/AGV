#!/usr/bin/env python
# Marc Simmonds
# IGVC 2022
""" 

    Detects all white colored road markings and publishes them as polygons. 

"""

from pickletools import uint8
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
import cv2
import std_msgs
from cv_bridge import CvBridge
import math
import numpy as np

IMG_WIDTH = 672
IMG_HEIGHT = 376

class road_marking_detect:


    def __init__(self):

        # Class members for depth camera image and flag for determining if message is sent
        self.img = None
        self.ready_depth = False
        self.depth_img = None
        self.bridge = CvBridge()
        self.frame = "choo_2/camera"

        rospy.init_node("road_marks", anonymous = False, log_level = rospy.INFO)

        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, callback=self.img_callback)
 
        self.depth_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, callback = self.depth_callback)

        self.pub = rospy.Publisher("bowser2/potholes", PolygonStamped, queue_size = 3)
    
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(1)

    def depth_callback(self, depth_msg):
        #self.depth_img = self.bridge.imgmsg_to_cv2(depth_msg)
        self.depth_img = np.frombuffer(depth_msg.data,dtype=np.uint8).reshape(depth_msg.height,depth_msg.width,-1)
        self.ready_depth = True

    def img_callback(self, img_msg):
        #cv_img = self.bridge.imgmsg_to_cv2(img_msg)
        cv_img = np.frombuffer(img_msg.data,dtype=np.uint8).reshape(img_msg.height,img_msg.width,-1)
        self.ready_img = True
        
        # Pre-process image 
        ret, cv_img = cv2.threshold(cv_img,125,255,0)
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        cv_img = cv2.blur(cv_img, (5,5))

        Ipt_A = [0, IMG_HEIGHT/2]
        Ipt_B = [0, IMG_HEIGHT-1]
        Ipt_C = [IMG_WIDTH-1, IMG_HEIGHT-1]
        Ipt_D = [IMG_WIDTH-1, IMG_HEIGHT/2]

        Opt_A = [0,0]
        Opt_B = [0, IMG_HEIGHT-1]
        Opt_C = [IMG_WIDTH-1, IMG_HEIGHT-1]
        Opt_D = [IMG_WIDTH-1,0]

        in_pts = np.float32([Ipt_A, Ipt_B, Ipt_C, Ipt_D])
        out_pts = np.float32([Opt_A, Opt_B, Opt_C, Opt_D])

        transform = cv2.getPerspectiveTransform(in_pts, out_pts)
        cv_img = cv2.warpPerspective(cv_img, transform, (IMG_WIDTH, IMG_HEIGHT), flags=cv2.INTER_LINEAR)

        contours, hierarchy = cv2.findContours(cv_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
            
        
        
        # Reverse transforms
        inv_trans = np.linalg.pinv(transform)
        contours = cv2.perspectiveTransform(contours, inv_trans)
        cv_img = cv2.perspectiveTransform(cv_img, inv_trans)

        # Reverse Image 
        cv2.drawContours(cv_img, contours, -1, (0,250,0), 3)
        self.show_image(cv_img)

    def isPothole(self, perimeter, area):
        
        if perimeter == 0 or area == 0:
            return False

        circularity = (4*math.pi*area)/(math.pow(perimeter,2))

        if circularity > 0.5 and 75<area<50000:
            return True
 
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
    def getMarks(self):

        while not rospy.is_shutdown():

            # If the image hasn't been published, we don't do anything
            if (not self.ready_img) and (not self.ready_depth):
                continue

            cv_img = self.img
            
            # Find the contours in the image
            # contours, hierarchy = cv2.findContours(cv_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # # Find contours
            
            # validContours = []
            # for i in contours:
                
            #     area = cv2.contourArea(i)
            #     perimeter = cv2.arcLength(i, True)
                
            # #    if self.isPothole(perimeter, area):
            # #       validContours.append(i)
             
            # if not len(validContours) <= 0:
            #     # Polygon
            #     for contour in validContours:
            #         p = PolygonStamped()
            #         # Point
            #         for point in contour:
            #             point = np.squeeze(point)
                        
            #             # Get locations of road markings
            #             try:
            #                 xPoint = self.getLocation(point, self.depth_img[point[1]][point[0]])
            #                 zPoint = self.depth_img[point[1]][point[0]]
            #                 p.polygon.points.append( Point32(x=xPoint, y=0.000, z = zPoint))
                        
            #             except IndexError:
            #                 continue
                        
            #             except TypeError:
            #                 continue

            #         p.header.stamp = rospy.Time.now()
            #         p.header.frame_id = self.frame
            #         self.pub.publish(p)


            # Show what's going on as a sanity check
            ##print(cv_img)
            ##cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
            
            #cv2.drawContours(cv_img, validContours, -1, (0,250,0), 3)

            ##self.show_image(cv_img)
            ##self.ready_img = False
            ##self.ready_depth = False

def main():

    img_detect_node = road_marking_detect()
    img_detect_node.getMarks()

if __name__=='__main__':

    main()