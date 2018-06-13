#!/usr/bin/env python

import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import Char
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.light_pub = rospy.Publisher("vision/light", Char, queue_size=10)
        self.image_sub = rospy.Subscriber("image/rgb", Image, self.callback)

    def callback(self,data):
        if (rospy.get_rostime() - data.header.stamp).to_sec() > 0.07:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Define limits for color detection
        lowerRed = np.array([0,168,67], dtype="uint8")
        upperRed = np.array([41,255,121], dtype="uint8")
        lowerGreen = np.array([55,82,0], dtype="uint8")
        upperGreen = np.array([92,160,103], dtype="uint8")
        radMin = 20
        light = 'N'

        # Convert the image to HSV and apply a blur
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        kSize = 9
        #gaussImg = cv2.GaussianBlur(hsvImg, (kSize,kSize), 0)

        # Apply masks for red and green and find the relevant contours
        redMask = cv2.inRange(hsvImg, lowerRed, upperRed)
        redMask = cv2.erode(redMask, None, iterations=2)
        redMask = cv2.dilate(redMask, None, iterations=2)
        redCnts = cv2.findContours(redMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        greenMask = cv2.inRange(hsvImg, lowerGreen, upperGreen)
        greenMask = cv2.erode(greenMask, None, iterations=2)
        greenMask = cv2.dilate(greenMask, None, iterations=2)
        greenCnts = cv2.findContours(greenMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]


        if len(redCnts) > 0:
            redC = max(redCnts, key=cv2.contourArea)
            ((redX,redy),redR) = cv2.minEnclosingCircle(redC)

            if redR > radMin:
                light = 'R'
        elif len(greenCnts) > 0:
            greenC = max(greenCnts, key=cv2.contourArea)
            ((greenX,greenY),greenR) = cv2.minEnclosingCircle(greenC)

            if greenR > radMin:
                light = 'G'

        self.light_pub.publish(ord(light))

def main():
    rospy.init_node('light_detection',anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__=="__main__":
    main()
        
