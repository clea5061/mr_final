#!/usr/bin/env python

import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import Char
from sensor_msgs.msg import Image
from cv_bridge impoty CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.light_pub = rospy.Publisher("vision/light", Char, queue_size=10)
        self.image_sub = rospy.Subscriber("image/rgb", Image, self.callback)

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Define limits for color detection
        lowerRed = np.array([0,0,0], dtype="uint8")
        upperRed = np.array([255,255,255], dtype="uint8")
        lowerGreen = np.array([0,0,0], dtype="uint8")
        upperGreen = np.array([255,255,255], dtype="uint8")
        radMin = 10
        light = 'N'

        # Convert the image to HSV and apply a blur
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        kSize = 9
        gaussImg = cv2.GaussianBlur(hsvImg, (kSize,kSize), 0)

        # Apply masks for red and green and find the relevant contours
        redMask = cv2.inRange(gaussImg, lowerRed, upperRed)
        redCnts = cv2.findCountours(redMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        greenMask = cv2.inRange(gaussImg, lowerGreen, upperGreen)
        greenCnts = cv2.findCountours(greenMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]


        if len(redCnts) > 0:
            redC = max(redCnts, key=cv2.contourArea)
            ((redX,redy),redR) = cv2.minEnclosingCircle(redC)

            if radius > radMin:
                light = 'R'
        
        elif len(greenCnts) > 0:
            greenC = max(greenCnts, key=cv2.contourArea)
            ((greenX,greenY),greenR) = cv2.minEnclosingCircle(greenC)

            if greenR > radMin:
                light = 'G'

        self.light_pub_publish(light)

def main():
    ic = image_converter()
    rospy.init_node('image_converter',anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__=="__main__":
    main()
        
