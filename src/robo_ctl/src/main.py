#!/usr/bin/env python

import rospy
from robot import Robot


def main():
    rospy.init_node('robo_control')
    Robot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()