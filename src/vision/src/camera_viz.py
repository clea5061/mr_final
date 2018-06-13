#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()


def canny_callback(data):
    img = bridge.imgmsg_to_cv2(data, "mono8")
    cv2.waitKey(1)
    cv2.imshow('canny', img)

def rgb_callback(data):
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.waitKey(1)
    cv2.imshow('rgb', img)


def main():
    rospy.init_node('cam_viz')
    rospy.Subscriber("image/canny", Image, canny_callback)
    rospy.Subscriber("image/rgb", Image, rgb_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
