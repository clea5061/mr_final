#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Webcam:

    def __init__(self):
        self.image_pub = rospy.Publisher("image/rgb", Image, queue_size=10)
        rospy.loginfo("Logging to node: [%s]", "image/rgb")
        self.bridge = CvBridge()

    def camera(self):
        cap = cv2.VideoCapture(0)
        rospy.loginfo("Camera activated")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        while not rospy.is_shutdown():
            ret, img = cap.read()
            try:
                img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                img_msg.header.stamp = rospy.Time.now()
                self.image_pub.publish(img_msg)
            except:
                pass
        cap.release()

def main():
    webcam = Webcam()
    rospy.init_node("webcam")
    webcam.camera()

if __name__ == '__main__':
    main()

