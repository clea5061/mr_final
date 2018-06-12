#!/usr/bin/env python

import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def roi(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask,vertices,255)
    masked = cv2.bitwise_and(img,mask)
    return masked

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image/canny", Image, queue_size=10)
        self.string_pub = rospy.Publisher("string/center", String, queue_size=10)
        self.image_sub = rospy.Subscriber("image/rgb", Image, self.callback)

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Define lane marking color limits
        lowerBlack = np.array([0, 0, 0], dtype="uint8")
        upperBlack = np.array([255, 255, 255], dtype="uint8")

        # Load a color image
        height,width = img.shape[:2]
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Gaussian blur for noise suppression
        kSize = 9
        gaussImg = cv2.GaussianBlur(grayImg,(kSize,kSize),0)

        # Edge detection
        lowThresh = 30
        highThresh = 150
        cannyImg = cv2.Canny(gaussImg,lowThresh,highThresh)

        # ROI filter
        top = 210
        bottom = 100
        vertices = np.array([[0,top],[0,bottom],[320,bottom],[320,top]], np.int32)
        roiImg = roi(cannyImg, [vertices])

        centerMask = np.zeros((height,width), np.uint8)
        laneTrack = np.zeros((top-bottom, 4), np.int)
        centerX = np.zeros((top-bottom), np.int)
        centerY = np.zeros((top-bottom), np.int)
        iteration = 0
        for i in range(bottom, top):
            track = 0
            for j in range(width):
                if roiImg.item(i,j) > 0 and track < 4 and (abs(j - laneTrack[iteration,track-1]) > 3):
                    if track == 0 and ((abs(j - laneTrack[iteration-1,0]) < 8) or laneTrack[iteration-1,0] == 0):
                        laneTrack[iteration,0] = j
                        track = track + 1
                    elif track == 1 and ((abs(j - laneTrack[iteration-1,1]) < 8) or laneTrack[iteration-1,1] == 0):
                        laneTrack[iteration,1] = j
                        track = track + 1
                    elif track == 2 and ((abs(j - laneTrack[iteration-1,2]) < 8) or laneTrack[iteration-1,2] == 0):
                        laneTrack[iteration,2] = j
                        track = track + 1
                    elif track == 3 and ((abs(j - laneTrack[iteration-1,3]) < 8) or laneTrack[iteration-1,3] == 0):
                        laneTrack[iteration,3] = j
                        track = track + 1
                        center = (laneTrack[iteration,2]+laneTrack[iteration,1])/2
                        centerX[iteration] = i
                        centerY[iteration] = center
                        centerMask.itemset((i,center),255)
            iteration = iteration + 1

        centerPoint = ("[" + str(centerX[top-15]) + " " + str(centerY[top-15]) + "]")
        self.string_pub.publish(centerPoint)

        finalImg = cv2.bitwise_or(roiImg, centerMask)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(finalImg, "bgr"))
        except CvBridgeError as e:
            print(e)

def main():
    ic = image_converter()
    rospy.init_node('image_converter',anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=="__main__":
    main()
