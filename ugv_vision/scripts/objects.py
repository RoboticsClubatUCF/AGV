#!/usr/bin/env python3

# Marc Simmonds
# IGVC 2022

"""
    This script is for object detection and recognition. 
    When i'm done, it will: 

    - Listen for depth camera messages
    - Find the obstacles in the image
    - Classify each one
    - Get the location in terms of x, y, and z (relative to the robot)
    - Publish the locations
"""

from cv_bridge.core import CvBridge
import rospy 
from sensor_msgs.msg import Image

class object_detect:

    # Class constructor
    def __init__(self):

        # Class fields, we want these to be accessible throughout the entire file
        self.depth = None
        self.img = None
        self.ready_depth = False
        self.ready_img = False
        self.bridge = CvBridge()
     
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/bowser2/bowser2_dc/image/image", Image, callback=self.img_callback)

        # Subscribe to depth image topic
        self.depth_sub = rospy.Subscriber("/bowser2/bowser2_dc/depth/image", Image, callback = self.depth_callback)

        # Initialize Node
        rospy.init_node("Obstacles", anonymous=True, log_level=rospy.INFO)

    def img_callback(self, img_msg):
        self.img = self.bridge.imgmsg_to_cv2(img_msg)
        self.ready_img = True

    def depth_callback(self, depth_msg):
        self.depth = self.bridge.imgmsg_to_cv2(depth_msg)
        self.ready_depth = True

            