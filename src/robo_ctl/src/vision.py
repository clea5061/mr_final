from math import sqrt,sin
import numpy as np
from geometry_msgs.msg import Twist

def lane_center_to_twist(center, speed):
    twst = Twist()
    z = 33.0
    curve = sqrt(center**2 + z**2)/(2*sin(np.pi/(1+center/160))*23.0)
    velocity = speed+(speed-(2*speed*curve))/(2*curve+1)
    if center < 0:
        twst.angular.x = velocity
        twst.angular.y = speed
    else if center > 0:
        twst.angular.y = velocity
        twst.angular.x = speed
    else:
        twst.angular.x = speed
        twst.angular.y = speed
    return twst