#!/usr/bin/env python

# ROS system imports
import rospy

import geometry_msgs.msg as geom

def main():


    rospy.init_node("poly_test", anonymous=True, log_level=rospy.DEBUG)

    poly_pub = rospy.Publisher("/potholes", geom.PolygonStamped, queue_size=1)

    points = []

    for i in range(100):
        for j in range(100):
            pt = geom.Point()
            pt.x = i / 100 + 1
            pt.y = j / 100 + 1
            pt.z = 0
            points.append(pt)

    poly = geom.PolygonStamped()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        poly.header.stamp = rospy.Time.now()
        poly.header.frame_id = "odom"
        poly.polygon.points = points

        poly_pub.publish(poly)

        rate.sleep()


if __name__=="__main__":
    main()